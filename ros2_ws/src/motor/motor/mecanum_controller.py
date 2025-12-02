#!/usr/bin/env python3
"""
Mecanum wheel controller with direct velocity mapping.
Simple, precise control without complex feedback loops.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Bool
from interfaces.srv import EmergencyStop
from .motor_driver import MotorDriver


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MecanumController(Node):
    """
    Direct velocity control for mecanum wheels.

    Mecanum wheel configuration (X-pattern):
        FL ↗    FR ↖     (front)
        RL ↖    RR ↗     (rear)

    Robot frame: +X forward, +Y left, +Z up
    """

    def __init__(self) -> None:
        super().__init__("mecanum_controller")

        # === Hardware parameters ===
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_addr", 0x34)
        self.declare_parameter("motor_type", None)
        self.declare_parameter("encoder_polarity", 0)

        # === Geometry ===
        self.declare_parameter("wheel_radius", 0.0485)  # meters
        self.declare_parameter("base_length", 0.26)  # wheelbase (m)
        self.declare_parameter("base_width", 0.25)  # track width (m)
        self.declare_parameter("cpr", 2480.0)  # encoder counts/rev

        # === Per-wheel calibration ===
        self.declare_parameter("wheel_scales", [1.0, 1.0, 1.0, 1.0])  # [FL, FR, RL, RR]
        self.declare_parameter("auto_load_calibration", True)
        self.declare_parameter(
            "calibration_file", "~/ros2_ws/config/mecanum_calibration.yaml"
        )

        # === Control parameters ===
        self.declare_parameter("velocity_scale", 25.0)  # m/s -> board units
        self.declare_parameter("angular_scale", 15.0)  # rad/s -> board units
        self.declare_parameter("cmd_vel_timeout", 0.25)  # s
        self.declare_parameter("cmd_deadband", 0.01)  # ignore tiny inputs
        self.declare_parameter("control_rate_hz", 100.0)  # Hz

        # === Acceleration limits for smooth motion ===
        self.declare_parameter("max_accel", 3.0)  # m/s²
        self.declare_parameter("max_decel", 6.0)  # m/s²
        self.declare_parameter("max_angular_accel", 4.0)  # rad/s²
        self.declare_parameter("max_angular_decel", 8.0)  # rad/s²

        # === Strafe compensation ===
        self.declare_parameter("strafe_compensation", 1.0)

        # === Safety ===
        self.declare_parameter("send_zero_on_start", True)
        self.declare_parameter("post_limit_deadband", 0.002)

        # Read parameters
        bus = int(self.get_parameter("i2c_bus").value)
        addr = int(self.get_parameter("i2c_addr").value)
        motor_type = self.get_parameter("motor_type").value
        enc_pol = int(self.get_parameter("encoder_polarity").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.Lx = float(self.get_parameter("base_length").value) / 2.0
        self.Ly = float(self.get_parameter("base_width").value) / 2.0
        self.cpr = float(self.get_parameter("cpr").value)

        ws_param = self.get_parameter("wheel_scales").value
        self.wheel_scales = list(ws_param) if isinstance(ws_param, (list, tuple)) else [1.0, 1.0, 1.0, 1.0]
        if len(self.wheel_scales) != 4:
            self.get_logger().warn("wheel_scales should have 4 values, using defaults")
            self.wheel_scales = [1.0, 1.0, 1.0, 1.0]
        # clamp unreasonable scales
        self.wheel_scales = [float(clamp(s, 0.5, 1.5)) for s in self.wheel_scales]

        auto_load = bool(self.get_parameter("auto_load_calibration").value)
        calib_file = str(self.get_parameter("calibration_file").value)
        if auto_load:
            self._load_saved_calibration(calib_file)

        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.cmd_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.cmd_deadband = float(self.get_parameter("cmd_deadband").value)

        self.max_accel = float(self.get_parameter("max_accel").value)
        self.max_decel = float(self.get_parameter("max_decel").value)
        self.max_ang_accel = float(self.get_parameter("max_angular_accel").value)
        self.max_ang_decel = float(self.get_parameter("max_angular_decel").value)

        self.strafe_comp = float(self.get_parameter("strafe_compensation").value)

        self.send_zero_on_start = bool(self.get_parameter("send_zero_on_start").value)
        self.post_limit_deadband = float(self.get_parameter("post_limit_deadband").value)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate_hz

        # Commanded velocities
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # Actual velocities (after accel limiting)
        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.actual_wz = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.prev_encoder_counts = None

        # Emergency stop state
        self.emergency_stopped = False

        # Initialize hardware
        try:
            self.driver = MotorDriver(
                bus=bus, addr=addr, motor_type=motor_type, encoder_polarity=enc_pol
            )
            if self.send_zero_on_start:
                try:
                    for _ in range(3):
                        self.driver.set_motor_speed([0, 0, 0, 0])
                except Exception as e:
                    self.get_logger().warn(f"Initial stop failed: {e}")
            self.get_logger().info(
                f"✓ Motor driver initialized (I2C bus {bus}, addr 0x{addr:02X})"
            )
        except Exception as e:
            self.get_logger().fatal(f"✗ Motor driver init failed: {e}")
            raise

        # ROS interfaces
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.pub_encoders = self.create_publisher(Int32MultiArray, "wheel_encoders", 100)
        self.pub_estop_state = self.create_publisher(Bool, "emergency_stop_state", 10)

        # Emergency stop service
        self.srv_estop = self.create_service(
            EmergencyStop,
            "/emergency_stop",
            self.emergency_stop_callback
        )

        # Control timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Publish e-stop state periodically
        self.estop_timer = self.create_timer(0.1, self.publish_estop_state)

        self.get_logger().info(
            f"2 Advanced mecanum controller ready | "
            f"Wheel radius: {self.wheel_radius:.4f}m | "
            f"Base: {self.Lx*2:.3f}×{self.Ly*2:.3f}m | "
            f"Wheel scales: {self.wheel_scales} | "
            f"Accel: {self.max_accel:.1f}/{self.max_decel:.1f} m/s² | "
            f"Control rate: {rate_hz:.0f}Hz"
        )

    def _load_saved_calibration(self, calib_file: str) -> None:
        import os
        import yaml

        calib_path = os.path.expanduser(calib_file)
        if not os.path.exists(calib_path):
            self.get_logger().info(f"No saved calibration found at {calib_path}")
            return

        try:
            with open(calib_path, "r") as f:
                calib_data = yaml.safe_load(f)

            if "wheel_scales" in calib_data:
                loaded_scales = calib_data["wheel_scales"]
                if isinstance(loaded_scales, (list, tuple)) and len(loaded_scales) == 4:
                    self.wheel_scales = [float(clamp(s, 0.5, 1.5)) for s in loaded_scales]
                    self.get_logger().info(
                        f"✅ Loaded calibration: wheel_scales={self.wheel_scales}"
                    )

            if "max_accel" in calib_data:
                self.max_accel = float(calib_data["max_accel"])
            if "max_decel" in calib_data:
                self.max_decel = float(calib_data["max_decel"])

            if "strafe_compensation" in calib_data:
                self.strafe_comp = float(calib_data["strafe_compensation"])

            self.get_logger().info(f"📁 Calibration loaded from: {calib_path}")

        except Exception as e:
            self.get_logger().warn(f"Failed to load calibration: {e}")

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service requests."""
        self.emergency_stopped = request.enable
        
        if self.emergency_stopped:
            # Immediately stop all motion
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0
            self.actual_vx = 0.0
            self.actual_vy = 0.0
            self.actual_wz = 0.0
            
            # Send stop command to motors multiple times for safety
            try:
                for _ in range(3):
                    self.driver.set_motor_speed([0, 0, 0, 0])
                response.success = True
                response.message = "Emergency stop activated - robot stopped"
                self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED")
            except Exception as e:
                response.success = False
                response.message = f"Failed to stop motors: {e}"
                self.get_logger().error(f"Emergency stop failed: {e}")
        else:
            response.success = True
            response.message = "Emergency stop deactivated - normal operation resumed"
            self.get_logger().info("✅ Emergency stop deactivated")
        
        response.is_stopped = self.emergency_stopped
        self.publish_estop_state()
        return response

    def publish_estop_state(self):
        """Publish current emergency stop state."""
        msg = Bool()
        msg.data = self.emergency_stopped
        self.pub_estop_state.publish(msg)

    def cmd_vel_callback(self, msg: Twist) -> None:
        # Ignore cmd_vel when emergency stopped
        if self.emergency_stopped:
            return
            
        self.last_cmd_time = self.get_clock().now()

        def apply_deadband(value: float) -> float:
            return 0.0 if abs(value) < self.cmd_deadband else value

        self.cmd_vx = apply_deadband(msg.linear.x)  # Negate to fix inverted X-axis
        self.cmd_vy = -apply_deadband(msg.linear.y)
        self.cmd_wz = apply_deadband(msg.angular.z)

    def apply_acceleration_limits(self) -> tuple:
        # Linear X
        vx_error = self.cmd_vx - self.actual_vx
        if abs(vx_error) < 0.001:
            vx = self.cmd_vx
        else:
            if (vx_error > 0 and self.actual_vx >= 0) or (vx_error < 0 and self.actual_vx <= 0):
                max_delta = self.max_accel * self.dt
            else:
                max_delta = self.max_decel * self.dt
            delta = clamp(vx_error, -max_delta, max_delta)
            vx = self.actual_vx + delta

        # Linear Y
        vy_error = self.cmd_vy - self.actual_vy
        if abs(vy_error) < 0.001:
            vy = self.cmd_vy
        else:
            if (vy_error > 0 and self.actual_vy >= 0) or (vy_error < 0 and self.actual_vy <= 0):
                max_delta = self.max_accel * self.dt
            else:
                max_delta = self.max_decel * self.dt
            delta = clamp(vy_error, -max_delta, max_delta)
            vy = self.actual_vy + delta

        # Angular Z
        wz_error = self.cmd_wz - self.actual_wz
        if abs(wz_error) < 0.001:
            wz = self.cmd_wz
        else:
            if (wz_error > 0 and self.actual_wz >= 0) or (wz_error < 0 and self.actual_wz <= 0):
                max_delta = self.max_ang_accel * self.dt
            else:
                max_delta = self.max_ang_decel * self.dt
            delta = clamp(wz_error, -max_delta, max_delta)
            wz = self.actual_wz + delta

        # Small post-limit deadband to prevent dithering
        if abs(vx) < self.post_limit_deadband:
            vx = 0.0
        if abs(vy) < self.post_limit_deadband:
            vy = 0.0
        if abs(wz) < self.post_limit_deadband:
            wz = 0.0

        self.actual_vx, self.actual_vy, self.actual_wz = vx, vy, wz
        return (vx, vy, wz)

    def mecanum_inverse_kinematics(self, vx: float, vy: float, wz: float) -> tuple:
        vy_compensated = vy * self.strafe_comp
        wheel_base_sum = self.Lx + self.Ly

        v_fl = vx - vy_compensated - wheel_base_sum * wz
        v_fr = vx + vy_compensated + wheel_base_sum * wz
        v_rl = vx + vy_compensated - wheel_base_sum * wz
        v_rr = vx - vy_compensated + wheel_base_sum * wz
        return (v_fl, v_fr, v_rl, v_rr)

    def velocity_to_board_units(self, velocity_mps: float, wheel_idx: int = 0) -> int:
        scale = self.wheel_scales[wheel_idx] if 0 <= wheel_idx < 4 else 1.0
        board_value = velocity_mps * self.velocity_scale * scale
        return int(clamp(round(board_value), -100, 100))

    def control_loop(self) -> None:
        # If emergency stopped, force all velocities to zero
        if self.emergency_stopped:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0
            self.actual_vx = 0.0
            self.actual_vy = 0.0
            self.actual_wz = 0.0
            
            # Send zero command to motors
            try:
                self.driver.set_motor_speed([0, 0, 0, 0])
            except Exception as e:
                self.get_logger().error(f"Failed to send stop in e-stop: {e}", throttle_duration_sec=1.0)
            
            self.read_and_publish_encoders()
            return
        
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0

        vx, vy, wz = self.apply_acceleration_limits()
        v_fl, v_fr, v_rl, v_rr = self.mecanum_inverse_kinematics(vx, vy, wz)

        cmd_fl = self.velocity_to_board_units(v_fl, 0)
        cmd_fr = self.velocity_to_board_units(v_fr, 1)
        cmd_rl = self.velocity_to_board_units(v_rl, 2)
        cmd_rr = self.velocity_to_board_units(v_rr, 3)

        motor_commands = [cmd_fl, cmd_fr, cmd_rl, cmd_rr]

        try:
            success = self.driver.set_motor_speed(motor_commands)
            if not success:
                self.get_logger().warn(
                    "Motor command failed (I2C write error)", throttle_duration_sec=1.0
                )
        except Exception as e:
            self.get_logger().error(
                f"Motor command failed: {e}", throttle_duration_sec=1.0
            )

        self.read_and_publish_encoders()

    def read_and_publish_encoders(self) -> None:
        try:
            encoders = self.driver.get_encoders()
            if encoders and len(encoders) == 4:
                msg = Int32MultiArray()
                msg.data = encoders
                self.pub_encoders.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Encoder read failed: {e}", throttle_duration_sec=2.0)

    def destroy_node(self) -> None:
        self.get_logger().info("Stopping motors...")
        try:
            for _ in range(3):
                self.driver.set_motor_speed([0, 0, 0, 0])
            self.driver.close()
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MecanumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()