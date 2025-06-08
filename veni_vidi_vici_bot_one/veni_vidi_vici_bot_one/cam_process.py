#!/usr/bin/env python3
"""
image_publisher.py  –  publishes raw camera frames on /image_in

Subscribes:  none
Publishes:   /image_in  (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.bridge = CvBridge()

        # Open default camera (/dev/video0). If your camera is /dev/video1, pass 1 instead.
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam (/dev/video0). Shutting down.")
            rclpy.shutdown()
            return

        # Optionally set frame width/height if needed:
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Publisher on /image_in:
        self.pub = self.create_publisher(Image, '/image_in', 10)

        # Timer to grab frames at 5 Hz (adjust as needed)
        timer_period = 1.0 / 20.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("ImagePublisher initialized, publishing at 20 Hz on /image_in.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # Convert OpenCV BGR image to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(img_msg)

    def destroy_node(self):
        # Release the camera before shutting down
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Catch Ctrl-C here. This prevents Python from throwing “init_import_site” errors.
        node.get_logger().info("KeyboardInterrupt received—shutting down cam_process node.")
    finally:
        # In any case (normal shutdown or Ctrl-C), clean up:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
