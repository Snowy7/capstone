#!/usr/bin/env python3
"""
Tool to measure actual encoder ticks per revolution (TPR).

Usage:
1. Run this script: ros2 run motor measure_tpr
2. Manually rotate ONE wheel exactly one full revolution
3. Read the reported tick count
4. Repeat a few times for accuracy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class TPRMeasurement(Node):
    def __init__(self):
        super().__init__('tpr_measurement')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("TPR (Ticks Per Revolution) Measurement Tool")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("Instructions:")
        self.get_logger().info("1. Make sure motors are OFF")
        self.get_logger().info("2. Note the initial encoder values below")
        self.get_logger().info("3. Manually rotate ONE wheel exactly 1 full revolution")
        self.get_logger().info("4. Note the final encoder values")
        self.get_logger().info("5. Calculate: TPR = |final - initial|")
        self.get_logger().info("")
        self.get_logger().info("Wheels: [FL, FR, RL, RR]")
        self.get_logger().info("-" * 60)
        
        self.sub = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10
        )
        
        self.initial_encoders = None
        self.previous_encoders = None
        
    def encoder_callback(self, msg):
        if len(msg.data) < 4:
            return
            
        encoders = list(msg.data)
        
        # Store initial values
        if self.initial_encoders is None:
            self.initial_encoders = encoders
            self.get_logger().info(f"Initial:  {encoders}")
            self.get_logger().info("")
            self.get_logger().info("Rotate a wheel now...")
            return
        
        # Only print if values changed significantly
        if self.previous_encoders is None or \
           any(abs(encoders[i] - self.previous_encoders[i]) > 5 for i in range(4)):
            
            # Calculate deltas from initial
            deltas = [encoders[i] - self.initial_encoders[i] for i in range(4)]
            
            self.get_logger().info(
                f"Current:  {encoders}  |  "
                f"Delta: [{deltas[0]:6d}, {deltas[1]:6d}, {deltas[2]:6d}, {deltas[3]:6d}]"
            )
            
            self.previous_encoders = encoders


def main(args=None):
    rclpy.init(args=args)
    node = TPRMeasurement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
