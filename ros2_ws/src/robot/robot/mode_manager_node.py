#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.srv import SwitchMode
import subprocess
import signal
import os
import time
import threading
from typing import List, Optional


class ManagedProcess:
    def __init__(self, name: str, popen: subprocess.Popen):
        self.name = name  # short tag like "SLAM", "LOC", "NAV"
        self.popen = popen
        self.stdout_thread: Optional[threading.Thread] = None
        self.stderr_thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.srv = self.create_service(SwitchMode, 'switch_mode', self.switch_mode_callback)

        self.current_mode = None
        self.processes: List[ManagedProcess] = []

        # Declare parameters
        self.declare_parameter('workspace_path', '~/ros2_ws')
        self.declare_parameter('slam_config', './src/robot/config/slam_toolbox.yaml')
        self.declare_parameter('amcl_config', './src/robot/config/amcl.yaml')
        self.declare_parameter('nav2_config', './src/robot/config/nav2_params.yaml')
        self.declare_parameter('default_map', './maps/my_map.yaml')

        # Get parameters
        self.workspace = os.path.expanduser(self.get_parameter('workspace_path').value)
        self.slam_config = os.path.join(self.workspace, self.get_parameter('slam_config').value.lstrip('./'))
        self.amcl_config = os.path.join(self.workspace, self.get_parameter('amcl_config').value.lstrip('./'))
        self.nav2_config = os.path.join(self.workspace, self.get_parameter('nav2_config').value.lstrip('./'))
        self.default_map = os.path.join(self.workspace, self.get_parameter('default_map').value.lstrip('./'))

        self.get_logger().info('Configuration paths:')
        self.get_logger().info(f'  SLAM config: {self.slam_config}')
        self.get_logger().info(f'  AMCL config: {self.amcl_config}')
        self.get_logger().info(f'  Nav2 config: {self.nav2_config}')
        self.get_logger().info(f'  Default map: {self.default_map}')
        self.get_logger().info('Mode Manager ready. Call /switch_mode ("mapping" | "navigation").')

    def switch_mode_callback(self, request, response):
        mode = request.mode.lower()
        map_name = request.map_name if request.map_name else self.default_map
        if map_name and not os.path.isabs(map_name):
            map_name = os.path.join(self.workspace, map_name.lstrip('./'))

        self.get_logger().info(f'Switching to mode: {mode}')

        self.stop_all_processes()
        time.sleep(2)

        if mode == 'mapping':
            ok = self.start_mapping_mode()
        elif mode == 'navigation':
            ok = self.start_navigation_mode(map_name)
        else:
            response.success = False
            response.message = f'Unknown mode "{mode}" (use "mapping" or "navigation").'
            return response

        if ok:
            self.current_mode = mode
            response.success = True
            response.message = f'Switched to {mode} mode'
        else:
            response.success = False
            response.message = f'Failed to start {mode} mode'
        return response

    # -------------------------
    # Process/logging utilities
    # -------------------------
    def _start_process(self, tag: str, cmd: list) -> Optional[ManagedProcess]:
        self.get_logger().info(f'[{tag}] CMD: {" ".join(cmd)}')
        try:
            p = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,  # line-buffered
                text=True,  # decode to str
                preexec_fn=os.setsid
            )
        except Exception as e:
            self.get_logger().error(f'[{tag}] Failed to start: {e}')
            return None

        mp = ManagedProcess(tag, p)
        mp.stdout_thread = threading.Thread(target=self._pipe_to_log, args=(mp, p.stdout, False), daemon=True)
        mp.stderr_thread = threading.Thread(target=self._pipe_to_log, args=(mp, p.stderr, True), daemon=True)
        mp.stdout_thread.start()
        mp.stderr_thread.start()

        self.processes.append(mp)
        self.get_logger().info(f'[{tag}] PID: {p.pid}')
        return mp

    def _pipe_to_log(self, mp: ManagedProcess, pipe, is_err: bool):
        # Streams each line to the node logger with the process tag
        logger = self.get_logger().error if is_err else self.get_logger().info
        tag = mp.name
        try:
            for line in iter(pipe.readline, ''):
                if mp.stop_flag.is_set():
                    break
                line = line.rstrip('\n')
                if line:
                    logger(f'[{tag}] {line}')
        except Exception as e:
            self.get_logger().warn(f'[{tag}] log thread error: {e}')
        finally:
            try:
                pipe.close()
            except Exception:
                pass

    def _is_alive(self, mp: ManagedProcess) -> bool:
        return mp.popen.poll() is None

    # ---------------
    # Mode starters
    # ---------------
    def start_mapping_mode(self) -> bool:
        self.get_logger().info('Starting SLAM mapping...')
        cmd = [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            f'slam_params_file:={self.slam_config}',
            'use_sim_time:=false'
        ]
        mp = self._start_process('SLAM', cmd)
        if mp is None:
            return False
        time.sleep(3)
        if not self._is_alive(mp):
            self.get_logger().error('[SLAM] Process exited prematurely')
            return False
        self.get_logger().info('SLAM mapping started')
        return True

    def start_navigation_mode(self, map_file: str) -> bool:
        self.get_logger().info(f'Starting Navigation with map: {map_file}')
        if not os.path.exists(map_file):
            self.get_logger().error(f'Map file not found: {map_file}')
            return False

        # 1) Localization
        loc_cmd = [
            'ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
            f'map:={map_file}',
            'use_sim_time:=false',
            f'params_file:={self.amcl_config}',
        ]
        loc = self._start_process('LOC', loc_cmd)
        if loc is None:
            return False

        time.sleep(4)
        if not self._is_alive(loc):
            self.get_logger().error('[LOC] Process exited prematurely')
            return False

        # 2) Navigation
        nav_cmd = [
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=false',
            f'params_file:={self.nav2_config}',
        ]
        nav = self._start_process('NAV', nav_cmd)
        if nav is None:
            return False

        time.sleep(3)
        if not (self._is_alive(loc) and self._is_alive(nav)):
            self.get_logger().error('[NAV] One or more processes exited prematurely')
            return False

        self.get_logger().info('Navigation stack started')
        return True

    # ---------------
    # Shutdown
    # ---------------
    def stop_all_processes(self):
        if not self.processes:
            return
        self.get_logger().info(f'Stopping {len(self.processes)} process(es)...')

        # Signal log threads to stop reading
        for mp in self.processes:
            mp.stop_flag.set()

        for mp in self.processes:
            try:
                if mp.popen.poll() is None:
                    os.killpg(os.getpgid(mp.popen.pid), signal.SIGTERM)
                    self.get_logger().info(f'[{mp.name}] SIGTERM sent (PID {mp.popen.pid})')
                    try:
                        mp.popen.wait(timeout=5)
                        self.get_logger().info(f'[{mp.name}] Terminated cleanly')
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn(f'[{mp.name}] Force killing')
                        os.killpg(os.getpgid(mp.popen.pid), signal.SIGKILL)
                        mp.popen.wait()
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().error(f'[{mp.name}] Stop error: {e}')

        # Join threads to be tidy
        for mp in self.processes:
            if mp.stdout_thread and mp.stdout_thread.is_alive():
                mp.stdout_thread.join(timeout=1.0)
            if mp.stderr_thread and mp.stderr_thread.is_alive():
                mp.stderr_thread.join(timeout=1.0)

        self.processes.clear()
        time.sleep(1)
        self.get_logger().info('All processes stopped')

    def destroy_node(self):
        self.get_logger().info('Shutting down mode manager...')
        self.stop_all_processes()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()