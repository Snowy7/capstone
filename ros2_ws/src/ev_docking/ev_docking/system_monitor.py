#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
import subprocess
import os

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(String, '/system_stats', 10)
        self.timer = self.create_timer(1.0, self.publish_stats) # 1Hz
        self.get_logger().info('System Monitor Node Started')

    def get_ros_nodes(self):
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            nodes = [n for n in result.stdout.split('\n') if n]
            return nodes
        except Exception as e:
            self.get_logger().error(f"Error getting node list: {e}")
            return []

    def get_voltage(self):
        try:
            # Output format: volt=0.8309V
            result = subprocess.run(['vcgencmd', 'measure_volts'], capture_output=True, text=True)
            return result.stdout.strip().replace("volt=", "")
        except Exception:
            return "N/A"

    def get_throttled(self):
        try:
            # Output format: throttled=0x50000
            result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True)
            hex_val = result.stdout.strip().replace("throttled=", "")
            return int(hex_val, 16)
        except Exception:
            return 0

    def get_logs(self):
        try:
            # Read last 20 lines of kern.log as dmesg substitute
            result = subprocess.run(['tail', '-n', '20', '/var/log/kern.log'], capture_output=True, text=True)
            return result.stdout.split('\n')
        except Exception:
            return ["Error reading logs"]

    def publish_stats(self):
        throttled_code = self.get_throttled()
        
        # Decode throttled bits
        # Bit 0: Under-voltage detected
        # Bit 1: Arm frequency capped
        # Bit 2: Currently throttled
        # Bit 3: Soft temperature limit active
        # Bit 16: Under-voltage has occurred
        # Bit 17: Arm frequency capped has occurred
        # Bit 18: Throttling has occurred
        # Bit 19: Soft temperature limit has occurred
        
        power_status = {
            "voltage": self.get_voltage(),
            "undervoltage_now": bool(throttled_code & 0x1),
            "throttled_now": bool(throttled_code & 0x4),
            "undervoltage_occurred": bool(throttled_code & 0x10000),
            "throttled_occurred": bool(throttled_code & 0x40000)
        }

        stats = {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "ram_percent": psutil.virtual_memory().percent,
            "ram_used_gb": round(psutil.virtual_memory().used / (1024**3), 2),
            "ram_total_gb": round(psutil.virtual_memory().total / (1024**3), 2),
            "disk_percent": psutil.disk_usage('/').percent,
            "nodes": self.get_ros_nodes(),
            "power": power_status,
            "logs": self.get_logs()
        }
        
        msg = String()
        msg.data = json.dumps(stats)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
