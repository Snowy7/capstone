#!/usr/bin/env python3
# teleop_keyboard.py
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP = """
Keyboard teleop (mecanum)
Controls:
  w/s: forward/back (x)
  a/d: left/right (y strafe)
  q/e: yaw left/right (z)
  z/x: decrease/increase speed scale
  r/f: decrease/increase angular scale
  c  : stop (zero all)
  h  : help
  ESC or Ctrl-C: exit

Current scales are shown each keypress.
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Parameters for base speeds
        self.declare_parameter("max_lin_speed", 0.8)   # m/s
        self.declare_parameter("max_ang_speed", 2.0)   # rad/s
        self.declare_parameter("publish_rate_hz", 30.0)

        self.max_lin = float(self.get_parameter("max_lin_speed").value)
        self.max_ang = float(self.get_parameter("max_ang_speed").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        # Runtime scales (0..1)
        self.lin_scale = 0.4
        self.ang_scale = 0.4

        # Command state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.publish_cmd)

        self.get_logger().info("Keyboard teleop started. Press 'h' for help.")
        print(HELP)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.vx * self.max_lin * self.lin_scale
        msg.linear.y = self.vy * self.max_lin * self.lin_scale
        msg.angular.z = self.wz * self.max_ang * self.ang_scale
        self.pub.publish(msg)

    def handle_key(self, ch: str):
        updated = False

        # Motion keys: set target directions (-1, 0, 1)
        if ch in ("w", "W"):
            self.vx = 1.0; updated = True
        elif ch in ("s", "S"):
            self.vx = -1.0; updated = True
        elif ch in ("a", "A"):
            self.vy = 1.0; updated = True
        elif ch in ("d", "D"):
            self.vy = -1.0; updated = True
        elif ch in ("q", "Q"):
            self.wz = 1.0; updated = True
        elif ch in ("e", "E"):
            self.wz = -1.0; updated = True

        # Speed scaling
        elif ch in ("z", "Z"):
            self.lin_scale = max(0.05, self.lin_scale - 0.05); updated = True
        elif ch in ("x", "X"):
            self.lin_scale = min(1.0, self.lin_scale + 0.05); updated = True
        elif ch in ("r", "R"):
            self.ang_scale = max(0.05, self.ang_scale - 0.05); updated = True
        elif ch in ("f", "F"):
            self.ang_scale = min(1.0, self.ang_scale + 0.05); updated = True

        # Stop
        elif ch in ("c", "C"):
            self.vx = 0.0; self.vy = 0.0; self.wz = 0.0; updated = True

        # Help
        elif ch in ("h", "H"):
            print(HELP)

        if updated:
            self.print_status()

    def stop(self):
        self.vx = 0.0; self.vy = 0.0; self.wz = 0.0
        self.publish_cmd()

    def print_status(self):
        lin = self.max_lin * self.lin_scale
        ang = self.max_ang * self.ang_scale
        print(f"[teleop] dir(x,y,wz)=({self.vx:+.0f},{self.vy:+.0f},{self.wz:+.0f}) "
              f"speeds: lin={lin:.2f} m/s ang={ang:.2f} rad/s")

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        while rclpy.ok():
            if sys.stdin.isatty():
                ch = getch()
                if ch == "\x1b":  # ESC
                    break
                node.handle_key(ch)
            else:
                # If not a tty, just spin and publish last command
                rclpy.spin_once(node, timeout_sec=0.1)
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()