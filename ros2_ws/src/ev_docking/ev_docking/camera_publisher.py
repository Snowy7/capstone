#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

import cv2
import numpy as np
import time
import threading
import queue
from typing import List, Optional


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # -----------------------------
        # Configuration / Tunables
        # -----------------------------
        self.camera_indices: List[int] = [0, 1]  # candidate indices
        self.camera_open_retries_per_index: int = 5
        self.camera_switch_cooldown_sec: float = 2.0

        self.max_capture_fails: int = 30  # consecutive read fails before switching
        self.capture_sleep_on_fail: float = 0.01
        self.capture_sleep_if_closed: float = 0.1

        self.target_width: int = 640
        self.target_height: int = 480
        self.target_fps: int = 30
        self.jpeg_quality: int = 80

        # -----------------------------
        # ROS Publishers
        # -----------------------------
        self.publisher_raw_ = self.create_publisher(
            Image, "/camera/image_raw", 10
        )

        qos_cam_info = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.camera_info_pub_ = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_cam_info
        )

        self.publisher_compressed_ = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 10
        )

        # -----------------------------
        # Internal State
        # -----------------------------
        self.cap: Optional[cv2.VideoCapture] = None
        self.current_camera_index_idx: int = 0  # index into self.camera_indices
        self.last_camera_switch_time: float = 0.0

        self.frame_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=1)
        self.running: bool = False
        self.capture_thread: Optional[threading.Thread] = None
        self.frame_count: int = 0

        # -----------------------------
        # Camera Info Message
        # -----------------------------
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.width = self.target_width
        self.camera_info_msg.height = self.target_height
        self.camera_info_msg.distortion_model = "plumb_bob"
        # Simple pinhole intrinsics — adjust if you have calibration
        fx = fy = float(self.target_width)
        cx = self.target_width / 2.0
        cy = self.target_height / 2.0
        self.camera_info_msg.k = np.array(
            [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ).tolist()
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.r = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        # -----------------------------
        # Initialize camera
        # -----------------------------
        self.get_logger().info("Initializing camera publisher with auto index switch...")
        opened = self.open_any_camera()
        if not opened:
            self.get_logger().error("No camera could be opened from the candidates.")
            # Node will stay alive but not publish; you can choose to shutdown instead.
        else:
            self.get_logger().info(
                f"Using camera index {self.get_current_camera_index()}"
            )

        # -----------------------------
        # Start capture thread
        # -----------------------------
        self.running = True
        self.capture_thread = threading.Thread(
            target=self.capture_loop, daemon=True
        )
        self.capture_thread.start()

        # -----------------------------
        # 30 Hz publication timer
        # -----------------------------
        self.timer = self.create_timer(1.0 / float(self.target_fps), self.timer_callback)

        self.get_logger().info("Optimized Camera Publisher Started (30Hz, multi-threaded)")

    # ============================================================
    # Camera Management
    # ============================================================
    def get_current_camera_index(self) -> int:
        return self.camera_indices[self.current_camera_index_idx]

    def open_camera(self, index: int) -> bool:
        """
        Try to open a specific camera index with a few retries, setting basic properties.
        """
        self.get_logger().info(f"Attempting to open camera at index {index}...")

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        for attempt in range(self.camera_open_retries_per_index):
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                # Configure properties
                cap.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc("M", "J", "P", "G"),
                )
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
                cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                # Not all backends support this
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass

                # Log actual properties
                actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                self.get_logger().info(
                    f"Camera index {index} opened: {actual_w}x{actual_h}@{actual_fps}fps"
                )

                self.cap = cap
                return True

            self.get_logger().warn(
                f"Camera {index} open attempt {attempt + 1} failed, retrying..."
            )
            cap.release()
            time.sleep(0.3)

        self.get_logger().error(f"Failed to open camera at index {index}")
        return False

    def open_any_camera(self) -> bool:
        """
        Try to open any camera from self.camera_indices, starting from current index.
        """
        n = len(self.camera_indices)
        for i in range(n):
            idx = (self.current_camera_index_idx + i) % n
            cam_index = self.camera_indices[idx]
            if self.open_camera(cam_index):
                self.current_camera_index_idx = idx
                self.last_camera_switch_time = time.time()
                return True

        return False

    def switch_to_next_camera(self):
        """
        Switch to the next camera index in self.camera_indices, with cooldown.
        """
        now = time.time()
        if now - self.last_camera_switch_time < self.camera_switch_cooldown_sec:
            # Avoid thrashing between cameras too fast
            return

        self.get_logger().warn(
            "Switching camera due to repeated capture failures or closure..."
        )

        n = len(self.camera_indices)
        self.current_camera_index_idx = (
            self.current_camera_index_idx + 1
        ) % n
        if self.open_any_camera():
            self.get_logger().info(
                f"Switched to camera index {self.get_current_camera_index()}"
            )
        else:
            self.get_logger().error("No camera available after attempting all indices.")
        self.last_camera_switch_time = now

    # ============================================================
    # Capture Thread
    # ============================================================
    def capture_loop(self):
        """
        Continuously read frames from the currently open camera.
        On repeated failures, attempt to switch camera indices.
        """
        consecutive_failures = 0

        while self.running:
            if self.cap is None or not self.cap.isOpened():
                # Camera is not currently open, try to open some index
                opened = self.open_any_camera()
                if not opened:
                    # Could not open any camera, wait a bit and retry
                    time.sleep(self.capture_sleep_if_closed)
                    continue
                else:
                    consecutive_failures = 0

            ret, frame = self.cap.read()
            if ret:
                consecutive_failures = 0
                # Keep only the latest frame
                try:
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    # Drop old frame
                    try:
                        _ = self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    # Put new one
                    try:
                        self.frame_queue.put_nowait(frame)
                    except queue.Full:
                        # If still full, just drop this frame
                        pass
            else:
                consecutive_failures += 1
                time.sleep(self.capture_sleep_on_fail)

                if consecutive_failures >= self.max_capture_fails:
                    self.get_logger().warn(
                        f"Capture failed {consecutive_failures} times; "
                        "attempting to switch camera."
                    )
                    self.switch_to_next_camera()
                    consecutive_failures = 0

    # ============================================================
    # ROS Timer Callback (Publishing)
    # ============================================================
    def timer_callback(self):
        try:
            frame = self.frame_queue.get_nowait()
        except queue.Empty:
            # No new frame — skip this cycle
            return

        try:
            timestamp = self.get_clock().now().to_msg()
            self.frame_count += 1

            if self.frame_count % 300 == 0:
                self.get_logger().info(f"Processed {self.frame_count} frames")

            # Publish CameraInfo (transient local; only if subscribers)
            if self.camera_info_pub_.get_subscription_count() > 0:
                self.camera_info_msg.header.stamp = timestamp
                self.camera_info_pub_.publish(self.camera_info_msg)

            # Publish compressed image (if subscribers)
            if self.publisher_compressed_.get_subscription_count() > 0:
                msg_compressed = CompressedImage()
                msg_compressed.header.stamp = timestamp
                msg_compressed.header.frame_id = "camera_link"
                msg_compressed.format = "jpeg"

                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                success, buffer = cv2.imencode(".jpg", frame, encode_param)
                if success:
                    msg_compressed.data = buffer.tobytes()
                    self.publisher_compressed_.publish(msg_compressed)
                else:
                    self.get_logger().warn("JPEG encoding failed; skipping compressed.")

            # Publish raw image (if subscribers)
            if self.publisher_raw_.get_subscription_count() > 0:
                msg = Image()
                msg.header.stamp = timestamp
                msg.header.frame_id = "camera_link"
                msg.height, msg.width = frame.shape[:2]
                msg.encoding = "bgr8"
                msg.is_bigendian = 0
                msg.step = msg.width * 3
                msg.data = frame.tobytes()
                self.publisher_raw_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    # ============================================================
    # Cleanup
    # ============================================================
    def destroy_node(self):
        self.running = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        self.cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()