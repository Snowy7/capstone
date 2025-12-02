#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from rclpy.callback_groups import ReentrantCallbackGroup

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Params
        self.declare_parameter('target_dist', 0.01)
        self.declare_parameter('linear_kp', 3.8)
        self.declare_parameter('angular_kp', 3.0)
        self.declare_parameter('tag_size', 0.029) # Size in meters
        
        self.target_dist = self.get_parameter('target_dist').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.tag_size = self.get_parameter('tag_size').value
        
        # Default Target ID
        self.target_tag_id = -1 
        self.is_docking = False
        
        # CV Setup
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera Params (fx, fy, cx, cy)
        self.camera_matrix = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=float)
        self.dist_coeffs = np.zeros((5,1))
        
        # Subs/Pubs
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Int32, '/target_dock_id', self.target_id_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_img_pub = self.create_publisher(CompressedImage, '/docking/debug_image/compressed', 10)
        
        self.dock_status_pub = self.create_publisher(String, '/docking/status', 10)
        
        self.get_logger().info('Docking Controller Started - Looking for ID 0')

    def target_id_callback(self, msg):
        self.target_tag_id = msg.data
        self.get_logger().info(f'Target changed to: {self.target_tag_id}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # --- FLIP FIX ---
        # 0  = Vertical Flip (Mirror upside down)
        # 1  = Horizontal Flip (Mirror left/right)
        # -1 = Both (Rotates image 180 degrees - Use this if camera is physically upside down)
        # frame = cv2.flip(frame, -1) 
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        detected_target = False
        distance = 0.0
        
        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                current_id = ids[i][0]
                current_corners = [corners[i]]
                
                # Get Pose
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    current_corners, self.tag_size, self.camera_matrix, self.dist_coeffs
                )
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                
                # Draw Axis
                try:
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.tag_size)
                except AttributeError:
                    cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.tag_size)

                # Draw Text
                x, y, z = tvec
                label = f"ID:{current_id} Z:{z:.2f}m"
                top_left = (int(corners[i][0][0][0]), int(corners[i][0][0][1]))
                
                # Text shadow (black) then text (yellow)
                cv2.putText(frame, label, (top_left[0]+1, top_left[1] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(frame, label, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Control Logic
                if current_id == self.target_tag_id:
                    detected_target = True
                    distance = float(z)
                    
                    twist = Twist()
                    if z > self.target_dist:
                        twist.linear.x = float(np.clip(self.linear_kp * (z - self.target_dist), -0.8, 0.8))
                        # IMPORTANT: Flipping the image also flips the X axis logic. 
                        # If the robot turns the wrong way, change -self.angular_kp to +self.angular_kp
                        twist.angular.z = float(np.clip(-self.angular_kp * x, -0.9, 0.0))
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        cv2.putText(frame, "DOCKED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                    
                    self.cmd_vel_pub.publish(twist)
                    self.is_docking = True

        if not detected_target and self.is_docking:
            self.cmd_vel_pub.publish(Twist())
            # Keep is_docking true for a bit or reset? For now let's reset if lost
            # self.is_docking = False 
        
        # Publish status
        status_msg = {
            "detected": detected_target,
            "target_id": int(self.target_tag_id),
            "distance": distance,
            "is_docking": self.is_docking
        }
        self.dock_status_pub.publish(String(data=json.dumps(status_msg)))
        
        self.debug_img_pub.publish(self.bridge.cv2_to_compressed_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
