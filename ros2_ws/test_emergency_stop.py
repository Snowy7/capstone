#!/usr/bin/env python3
"""
Emergency Stop Test Script
Tests the emergency stop service functionality
"""

import rclpy
from rclpy.node import Node
from interfaces.srv import EmergencyStop
from std_msgs.msg import Bool
import time


class EmergencyStopTester(Node):
    def __init__(self):
        super().__init__('emergency_stop_tester')
        
        self.estop_client = self.create_client(EmergencyStop, '/emergency_stop')
        self.state_sub = self.create_subscription(
            Bool,
            '/emergency_stop_state',
            self.state_callback,
            10
        )
        
        self.current_state = None
        
        while not self.estop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency_stop service...')
    
    def state_callback(self, msg):
        self.current_state = msg.data
        state_str = "STOPPED" if msg.data else "RUNNING"
        self.get_logger().info(f'Current E-Stop State: {state_str}')
    
    def call_estop(self, enable):
        request = EmergencyStop.Request()
        request.enable = enable
        
        action = "ACTIVATE" if enable else "DEACTIVATE"
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'Test: {action} Emergency Stop')
        self.get_logger().info(f'{"="*50}')
        
        future = self.estop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        if response:
            self.get_logger().info(f'✅ Success: {response.success}')
            self.get_logger().info(f'📊 State: {"STOPPED" if response.is_stopped else "RUNNING"}')
            self.get_logger().info(f'💬 Message: {response.message}')
        else:
            self.get_logger().error('❌ Service call failed')
        
        return response
    
    def run_tests(self):
        self.get_logger().info('\n🧪 Starting Emergency Stop Tests...\n')
        time.sleep(1)
        
        # Test 1: Activate E-Stop
        self.get_logger().info('Test 1: Activate Emergency Stop')
        response = self.call_estop(True)
        if response and response.success and response.is_stopped:
            self.get_logger().info('✅ Test 1 PASSED\n')
        else:
            self.get_logger().error('❌ Test 1 FAILED\n')
        
        time.sleep(2)
        
        # Test 2: Try to activate again (should remain stopped)
        self.get_logger().info('Test 2: Activate again (idempotent)')
        response = self.call_estop(True)
        if response and response.success and response.is_stopped:
            self.get_logger().info('✅ Test 2 PASSED\n')
        else:
            self.get_logger().error('❌ Test 2 FAILED\n')
        
        time.sleep(2)
        
        # Test 3: Deactivate E-Stop
        self.get_logger().info('Test 3: Deactivate Emergency Stop')
        response = self.call_estop(False)
        if response and response.success and not response.is_stopped:
            self.get_logger().info('✅ Test 3 PASSED\n')
        else:
            self.get_logger().error('❌ Test 3 FAILED\n')
        
        time.sleep(2)
        
        # Test 4: Deactivate again (should remain running)
        self.get_logger().info('Test 4: Deactivate again (idempotent)')
        response = self.call_estop(False)
        if response and response.success and not response.is_stopped:
            self.get_logger().info('✅ Test 4 PASSED\n')
        else:
            self.get_logger().error('❌ Test 4 FAILED\n')
        
        self.get_logger().info('\n🎉 All tests completed!')


def main(args=None):
    rclpy.init(args=args)
    
    tester = EmergencyStopTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
