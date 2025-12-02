#!/usr/bin/env python3
# mecanum_odometry.py
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_q(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class MecanumOdometry(Node):
    def __init__(self) -> None:
        super().__init__("mecanum_odometry")

        # Parameters
        self.declare_parameter("wheel_radius", 0.0485)  # meters
        self.declare_parameter("base_length", 0.26)  # Lx*2
        self.declare_parameter("base_width", 0.25)  # Ly*2
        self.declare_parameter("ticks_per_rev", 2480)  # post-gearbox
        self.declare_parameter("encoder_noise_threshold", 2)  # ticks
        self.declare_parameter("scales", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)

        # Robustness params
        self.declare_parameter("max_dt", 0.3)  # s, max integration step if using wall-time dt
        self.declare_parameter("max_wheel_speed", 5.0)  # m/s physical limit
        self.declare_parameter("reject_spike_factor", 5.0)  # multiplier over per-cycle limit
        self.declare_parameter("unwrap_32bit", False)  # set True if MCU counters wrap uint32

        # Covariance parameters (higher values = more uncertainty)
        # Defaults set low as odometry is reliable
        self.declare_parameter("pose_covariance_x", 0.05)       # m²
        self.declare_parameter("pose_covariance_y", 0.05)       # m²
        self.declare_parameter("pose_covariance_theta", 0.05)   # rad²
        self.declare_parameter("twist_covariance_x", 0.05)      # (m/s)²
        self.declare_parameter("twist_covariance_y", 0.05)      # (m/s)²
        self.declare_parameter("twist_covariance_theta", 0.05)  # (rad/s)²

        # Read params
        self.R = float(self.get_parameter("wheel_radius").value)
        self.Lx = float(self.get_parameter("base_length").value) / 2.0
        self.Ly = float(self.get_parameter("base_width").value) / 2.0
        self.a = self.Lx + self.Ly
        test = float(self.get_parameter("ticks_per_rev").value)
        self.get_logger().info("test value: {}".format(test))
        self.TPR = float(self.get_parameter("ticks_per_rev").value)
        self.encoder_threshold = int(self.get_parameter("encoder_noise_threshold").value)

        scales_param = self.get_parameter("scales").value
        self.scales: List[float] = list(scales_param) if isinstance(scales_param, (list, tuple)) else [1.0, 1.0, 1.0, 1.0]
        if len(self.scales) != 4:
            self.get_logger().warn("scales should have 4 values; defaulting to 1.0")
            self.scales = [1.0, 1.0, 1.0, 1.0]

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.dt = 1.0 / float(self.get_parameter("publish_rate_hz").value)

        self.max_dt = float(self.get_parameter("max_dt").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.reject_spike_factor = float(self.get_parameter("reject_spike_factor").value)
        self.unwrap_32bit = bool(self.get_parameter("unwrap_32bit").value)

        # Read covariance parameters
        self.cov_pose_x = float(self.get_parameter("pose_covariance_x").value)
        self.cov_pose_y = float(self.get_parameter("pose_covariance_y").value)
        self.cov_pose_th = float(self.get_parameter("pose_covariance_theta").value)
        self.cov_twist_x = float(self.get_parameter("twist_covariance_x").value)
        self.cov_twist_y = float(self.get_parameter("twist_covariance_y").value)
        self.cov_twist_th = float(self.get_parameter("twist_covariance_theta").value)

        # Precompute tick limits
        circ = 2.0 * math.pi * self.R
        ticks_per_sec_limit = (self.max_wheel_speed / circ) * self.TPR * 1.5
        self.ticks_per_cycle_limit = ticks_per_sec_limit * self.dt  # conservative

        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_enc: Optional[List[int]] = None

        # Current velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # ROS I/O
        self.sub = self.create_subscription(
            Int32MultiArray, "wheel_encoders", self.enc_cb, 200
        )
        self.pub = self.create_publisher(Odometry, "odom", 100)
        self.tfb = TransformBroadcaster(self)

        # Timer for continuous publishing at fixed rate
        self.timer = self.create_timer(self.dt, self.publish_periodic)

        self.get_logger().info(
            f"mecanum_odometry: R={self.R:.4f} Lx={self.Lx:.4f} Ly={self.Ly:.4f} "
            f"TPR={self.TPR:.1f} frames {self.odom_frame}->{self.base_frame} @ {1.0/self.dt:.0f}Hz"
        )

    def _unwrap32_diff(self, prev: int, now: int) -> int:
        # Treat values as unsigned 32-bit tick counters
        diff = now - prev
        # convert to range [-2^31, 2^31-1]
        if diff > 2**31 - 1:
            diff -= 2**32
        elif diff < -2**31:
            diff += 2**32
        return diff

    def enc_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 4:
            return

        curr = [int(msg.data[i]) for i in range(4)]  # [FL, FR, RL, RR]
        if self.prev_enc is None:
            self.prev_enc = curr
            return

        if self.unwrap_32bit:
            d_ticks = [self._unwrap32_diff(self.prev_enc[i], curr[i]) for i in range(4)]
        else:
            d_ticks = [curr[i] - self.prev_enc[i] for i in range(4)]

        # Small stationary noise: zero velocity, update prev, no integration
        if all(abs(dt) < self.encoder_threshold for dt in d_ticks):
            self.prev_enc = curr
            self.vx = self.vy = self.wz = 0.0
            return

        # Reject implausible spikes
        limit = max(10.0, self.ticks_per_cycle_limit)
        if any(abs(dt) > self.reject_spike_factor * limit for dt in d_ticks):
            self.get_logger().warn(f"Encoder spike rejected, d_ticks={d_ticks}")
            self.prev_enc = curr
            self.vx = self.vy = self.wz = 0.0
            return

        self.prev_enc = curr

        # ticks -> wheel angle increments (rad) with per-wheel scale
        s = (2.0 * math.pi) / self.TPR
        dphi = [(d_ticks[i] * s) * float(self.scales[i]) for i in range(4)]
        dphi_fl, dphi_fr, dphi_rl, dphi_rr = dphi

        # Forward kinematics (mecanum X configuration)
        k = self.R / 4.0
        dx_b = k * (dphi_fl + dphi_fr + dphi_rl + dphi_rr)  # Negate to fix inverted X-axis
        dy_b = -k * (-dphi_fl + dphi_fr + dphi_rl - dphi_rr)
        dth = (self.R / (4.0 * self.a)) * (-dphi_fl + dphi_fr - dphi_rl + dphi_rr)

        # Rotate body increments into world
        prev_x, prev_y = self.x, self.y
        c, s_ = math.cos(self.th), math.sin(self.th)
        self.x += c * dx_b - s_ * dy_b
        self.y += s_ * dx_b + c * dy_b
        self.th = wrap_pi(self.th + dth)

        # Step warning if unusually large
        step = math.hypot(self.x - prev_x, self.y - prev_y)
        if step > 0.5:
            self.get_logger().warn(
                f"Large odom step {step:.2f} m; ticks={d_ticks}, dphi={tuple(round(v,3) for v in dphi)}"
            )

        # Update velocity state for continuous publishing (use fixed dt)
        self.vx = dx_b / self.dt
        self.vy = dy_b / self.dt
        self.wz = dth / self.dt

    def publish_periodic(self) -> None:
        # Use same timestamp for odom and TF
        stamp = self.get_clock().now().to_msg()
        self.publish_odom(self.vx, self.vy, self.wz, stamp)
        self.publish_tf(stamp)

    def publish_odom(self, vx: float, vy: float, wz: float, stamp=None) -> None:
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_q(self.th)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz

        # Use configurable covariances
        msg.pose.covariance[0] = self.cov_pose_x    # x position
        msg.pose.covariance[7] = self.cov_pose_y    # y position
        msg.pose.covariance[35] = self.cov_pose_th  # theta rotation

        msg.twist.covariance[0] = self.cov_twist_x   # x velocity
        msg.twist.covariance[7] = self.cov_twist_y   # y velocity
        msg.twist.covariance[35] = self.cov_twist_th # angular velocity

        self.pub.publish(msg)

    def publish_tf(self, stamp=None) -> None:
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_q(self.th)
        self.tfb.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()