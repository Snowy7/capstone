#!/usr/bin/env python3
import os
import json
import glob
import time
import math
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from interfaces.srv import (
    SetMode,
    ListMaps,
    LoadMap,
    SaveMap,
    NavigateToGoal,
    SaveParkingSpot,
    ListParkingSpots,
    DeleteParkingSpot,
)

from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class StateManager(Node):
    def __init__(self):
        super().__init__("state_manager")

        # Parameters
        self.declare_parameter("maps_dir", "~/ros2_ws/maps")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("mode_manager_service", "/switch_mode")
        self.declare_parameter("workspace_path", "~/ros2_ws")

        self.maps_dir = os.path.expanduser(
            self.get_parameter("maps_dir").value
        )
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.mode_mgr_srv_name = self.get_parameter(
            "mode_manager_service"
        ).value
        self.workspace = os.path.expanduser(
            self.get_parameter("workspace_path").value
        )
        os.makedirs(self.maps_dir, exist_ok=True)

        # Publishers
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        self.target_dock_pub = self.create_publisher(Int32, "/target_dock_id", 10)

        # TF listener (for publishing /robot_pose)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for Nav2
        self.nav_action = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

        # Client for mode manager
        from interfaces.srv import SwitchMode
        self.mode_manager_client = self.create_client(
            SwitchMode, self.mode_mgr_srv_name
        )

        # Services
        self.create_service(SetMode, "/set_mode", self.handle_set_mode)
        self.create_service(ListMaps, "/list_maps", self.handle_list_maps)
        self.create_service(LoadMap, "/load_map", self.handle_load_map)
        self.create_service(SaveMap, "/save_map", self.handle_save_map)
        self.create_service(
            NavigateToGoal, "/navigate_to_goal", self.handle_nav_goal
        )
        self.create_service(
            SaveParkingSpot,
            "/save_parking_spot",
            self.handle_save_parking_spot,
        )
        self.create_service(
            ListParkingSpots,
            "/list_parking_spots",
            self.handle_list_parking_spots,
        )
        self.create_service(
            DeleteParkingSpot,
            "/delete_parking_spot",
            self.handle_delete_parking_spot,
        )

        # Timers
        self.create_timer(0.2, self.publish_state)  # 5Hz update rate

        # State
        self.current_mode = "mapping"  # default until set
        self.current_map_name = ""

        self.get_logger().info(
            f"StateManager online. maps_dir={self.maps_dir}"
        )

    # -------------------------
    # Helpers
    # -------------------------
    def _maps(self):
        # Return base names (without .yaml)
        files = sorted(glob.glob(os.path.join(self.maps_dir, "*.yaml")))
        return [os.path.splitext(os.path.basename(p))[0] for p in files]

    def _parking_file(self, map_name: str):
        base = os.path.splitext(map_name)[0]
        return os.path.join(self.maps_dir, f"{base}_parking.json")

    def _load_parking(self, map_name: str):
        fp = self._parking_file(map_name)
        if os.path.exists(fp):
            with open(fp, "r") as f:
                return json.load(f)
        return {"spots": []}

    def _save_parking(self, map_name: str, data):
        fp = self._parking_file(map_name)
        with open(fp, "w") as f:
            json.dump(data, f, indent=2)

    def _map_path_from_name(self, map_name: str) -> str:
        if os.path.isabs(map_name):
            return map_name
        return os.path.join(self.maps_dir, f"{map_name}.yaml")

    # -------------------------
    # Timers
    # -------------------------
    def publish_state(self):
        """Publish unified robot state as JSON"""
        state = {
            "mode": self.current_mode,
            "map_name": self.current_map_name or "none",
            "nav2_ready": self.nav_action.server_is_ready(),
            "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "timestamp": time.time()
        }
        
        # Try to get current pose
        try:
            if self.tf_buffer.can_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            ):
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time()
                )
                state["pose"]["x"] = t.transform.translation.x
                state["pose"]["y"] = t.transform.translation.y
                
                # Calculate yaw from quaternion
                q = t.transform.rotation
                yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z)
                )
                state["pose"]["theta"] = yaw
        except Exception:
            pass
        
        # Publish as JSON string
        self.state_pub.publish(String(data=json.dumps(state)))

    # -------------------------
    # Service handlers
    # -------------------------
    def handle_set_mode(self, req: SetMode.Request, res: SetMode.Response):
        mode = req.mode.lower().strip()
        if mode not in ("mapping", "navigation"):
            res.success = False
            res.message = 'Mode must be "mapping" or "navigation".'
            res.current_mode = self.current_mode
            return res

        # Check if mode manager client is ready (wait up to 5 seconds)
        self.get_logger().info(f"Waiting for mode manager service...")
        if not self.mode_manager_client.wait_for_service(timeout_sec=5.0):
            res.success = False
            res.message = "Mode manager service not available after 5s timeout."
            res.current_mode = self.current_mode
            self.get_logger().error(f"Cannot connect to {self.mode_mgr_srv_name}")
            return res
        
        self.get_logger().info(f"Mode manager available, requesting switch to {mode} with map: {req.map_name}")

        from interfaces.srv import SwitchMode
        req2 = SwitchMode.Request()
        req2.mode = mode
        req2.map_name = self._map_path_from_name(req.map_name) if req.map_name else ""

        # Call async and set up callback to publish mode when complete
        future = self.mode_manager_client.call_async(req2)
        self.get_logger().info(f"Async call sent to mode manager")
        
        def mode_switch_callback(fut):
            try:
                result = fut.result()
                if result and result.success:
                    self.current_mode = mode
                    if mode == "navigation":
                        # Extract just the base name without path and extension
                        map_base = os.path.splitext(os.path.basename(req.map_name))[0] if req.map_name else ""
                        self.current_map_name = map_base
                    else:
                        self.current_map_name = ""
                    self.get_logger().info(f"✓ Mode successfully switched to {mode}")
                else:
                    msg = result.message if result else "Mode switch failed"
                    self.get_logger().error(f"✗ Mode switch failed: {msg}")
            except Exception as e:
                self.get_logger().error(f"✗ Mode switch callback error: {e}")
        
        future.add_done_callback(mode_switch_callback)
        
        # Return immediately with success (switching in progress)
        res.success = True
        res.message = f"Switching to {mode} mode..."
        res.current_mode = self.current_mode  # Return current mode before switch
        self.get_logger().info(f"Returning immediate response, mode switch in progress")
        return res

    def handle_list_maps(self, _: ListMaps.Request, res: ListMaps.Response):
        maps = self._maps()
        res.success = True
        res.message = f"{len(maps)} map(s)"
        res.maps = maps
        return res

    def handle_load_map(self, req: LoadMap.Request, res: LoadMap.Response):
        # Switch to navigation mode with map
        set_req = SetMode.Request()
        set_res = SetMode.Response()
        set_req.mode = "navigation"
        set_req.map_name = req.map_name
        set_res = self.handle_set_mode(set_req, set_res)

        res.success = set_res.success
        res.message = set_res.message
        res.file_path = self._map_path_from_name(req.map_name)
        return res

    def handle_save_map(self, req: SaveMap.Request, res: SaveMap.Response):
        # Use map_saver_cli to write map into maps_dir
        name = req.map_name.strip() or time.strftime("map_%Y%m%d_%H%M%S")
        out = os.path.join(self.maps_dir, name)
        cmd = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            out,
        ]
        try:
            proc = subprocess.run(
                cmd, check=True, capture_output=True, text=True, timeout=60
            )
            res.success = True
            res.message = proc.stdout.strip() or "Map saved"
            res.file_path = f"{out}.yaml"
        except subprocess.CalledProcessError as e:
            res.success = False
            res.message = e.stderr or str(e)
            res.file_path = ""
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.file_path = ""
        return res

    def handle_nav_goal(
        self, req: NavigateToGoal.Request, res: NavigateToGoal.Response
    ):
        if not self.nav_action.server_is_ready():
            res.success = False
            res.message = "Nav2 action server not ready."
            return res

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = req.x
        goal.pose.pose.position.y = req.y
        goal.pose.pose.orientation.z = math.sin(req.theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(req.theta / 2.0)

        self.get_logger().info(
            f"NavigateToPose x={req.x:.2f}, y={req.y:.2f}, "
            f"theta={req.theta:.2f}, label={req.label}"
        )

        # Publish target dock ID if label is an integer
        try:
            dock_id = int(req.label)
            self.target_dock_pub.publish(Int32(data=dock_id))
            self.get_logger().info(f"Published target dock ID: {dock_id}")
        except ValueError:
            self.get_logger().warn(f"Parking label '{req.label}' is not an integer ID, ignoring for docking.")
            self.target_dock_pub.publish(Int32(data=-1))

        # Send goal and return immediately
        send_future = self.nav_action.send_goal_async(goal)
        
        def goal_response_callback(future):
            handle = future.result()
            if not handle or not handle.accepted:
                self.get_logger().error("Goal rejected by Nav2")
            else:
                self.get_logger().info("Goal accepted by Nav2, robot navigating...")
        
        send_future.add_done_callback(goal_response_callback)
        
        # Return immediately
        res.success = True
        res.message = "Navigation goal sent to robot"
        return res

    def handle_save_parking_spot(
        self, req: SaveParkingSpot.Request, res: SaveParkingSpot.Response
    ):
        data = self._load_parking(req.map_name)
        # new or update by spot_id
        if req.spot_id:
            # update existing
            for s in data["spots"]:
                if s["spot_id"] == req.spot_id:
                    s.update(
                        dict(
                            label=req.label,
                            map_name=req.map_name,
                            x=req.x,
                            y=req.y,
                            theta=req.theta,
                        )
                    )
                    break
        else:
            sid = f"spot_{int(time.time())}"
            data["spots"].append(
                dict(
                    spot_id=sid,
                    label=req.label,
                    map_name=req.map_name,
                    x=req.x,
                    y=req.y,
                    theta=req.theta,
                )
            )
            req.spot_id = sid

        self._save_parking(req.map_name, data)
        res.success = True
        res.message = "Saved"
        res.spot_id = req.spot_id
        return res

    def handle_list_parking_spots(
        self, req: ListParkingSpots.Request, res: ListParkingSpots.Response
    ):
        data = self._load_parking(req.map_name)
        res.success = True
        res.message = f"{len(data['spots'])} spot(s)"
        res.spot_ids = [s["spot_id"] for s in data["spots"]]
        res.labels = [s["label"] for s in data["spots"]]
        res.map_names = [s["map_name"] for s in data["spots"]]
        res.x = [float(s["x"]) for s in data["spots"]]
        res.y = [float(s["y"]) for s in data["spots"]]
        res.theta = [float(s["theta"]) for s in data["spots"]]
        return res

    def handle_delete_parking_spot(
        self, req: DeleteParkingSpot.Request, res: DeleteParkingSpot.Response
    ):
        # Search all map files to allow delete by id without map context
        for yaml in glob.glob(os.path.join(self.maps_dir, "*.yaml")):
            base = os.path.splitext(os.path.basename(yaml))[0]
            data = self._load_parking(base)
            before = len(data["spots"])
            data["spots"] = [
                s for s in data["spots"] if s["spot_id"] != req.spot_id
            ]
            if len(data["spots"]) != before:
                self._save_parking(base, data)
                res.success = True
                res.message = "Deleted"
                return res
        res.success = False
        res.message = "Spot not found"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()