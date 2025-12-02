#!/usr/bin/env python3
# encoders_to_joint_state.py (hardened)
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

class EncodersToJointState(Node):
    def __init__(self):
        super().__init__("encoders_to_joint_state")

        self.declare_parameter("ticks_per_rev", 2480.0)
        self.declare_parameter("scales", [1.0, 1.0, 1.0, 1.0])  # FL FR RL RR
        self.declare_parameter(
            "joint_names",
            ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"],
        )
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("unwrap_32bit", False)
        self.declare_parameter("encoder_noise_threshold", 0)  # ticks
        self.declare_parameter("reset_on_stall", False)       # keep angles if no motion

        self.tpr = float(self.get_parameter("ticks_per_rev").value)
        sparam = self.get_parameter("scales").value
        self.scales: List[float] = (
            list(sparam) if isinstance(sparam, (list, tuple)) else [1.0] * 4
        )
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.unwrap_32bit = bool(self.get_parameter("unwrap_32bit").value)
        self.noise_thr = int(self.get_parameter("encoder_noise_threshold").value)
        self.reset_on_stall = bool(self.get_parameter("reset_on_stall").value)

        self.prev_counts: Optional[List[int]] = None
        self.angles = [0.0, 0.0, 0.0, 0.0]
        self.initialized = False

        self.sub = self.create_subscription(Int32MultiArray, "wheel_encoders", self.cb, 20)
        self.pub = self.create_publisher(JointState, "joint_states", 20)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            f"encoders_to_joint_state: TPR={self.tpr}, unwrap32={self.unwrap_32bit}, names={self.joint_names}"
        )

    def _unwrap32_diff(self, prev: int, now: int) -> int:
        diff = now - prev
        if diff > 2**31 - 1:
            diff -= 2**32
        elif diff < -2**31:
            diff += 2**32
        return diff

    def cb(self, msg: Int32MultiArray):
        if len(msg.data) < 4:
            return
        curr = [int(v) for v in msg.data[:4]]

        if self.prev_counts is None:
            self.prev_counts = curr
            return

        if self.unwrap_32bit:
            d = [self._unwrap32_diff(self.prev_counts[i], curr[i]) for i in range(4)]
        else:
            d = [curr[i] - self.prev_counts[i] for i in range(4)]
        self.prev_counts = curr

        # Ignore tiny jitter
        if all(abs(di) <= self.noise_thr for di in d):
            # Do NOT reset angles here. Just keep last.
            return

        # Convert ticks to radians and accumulate
        step = (2.0 * math.pi) / self.tpr
        for i in range(4):
            self.angles[i] += d[i] * step * float(self.scales[i])

        self.initialized = True

    def publish(self):
        if not self.initialized:
            return  # don't publish zeros before first real delta
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.angles
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = EncodersToJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()