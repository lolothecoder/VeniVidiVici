#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node          import Node
from sensor_msgs.msg     import Image
from std_msgs.msg   import Float32
from cv_bridge           import CvBridge

BRIGHT_THR = 185          # 0–255 … raise/lower to taste

# image resolution (px)
IMG_W = 640

class DetectButton(Node):
    def __init__(self):
        super().__init__('detect_button')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/image_in', self.callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.button_pub = self.create_publisher(Float32, '/button_pose', 1)

        self.get_logger().info('Button Detection Initialised')

    def callback(self, msg: Image):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        lower_white = np.array([50,   50,   100], dtype=np.uint8)
        upper_white = np.array([255, 255,  255], dtype=np.uint8)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # create mask for white
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h = mask.shape[0]              # image height in pixels
        mask[h // 2 : , :] = 0
        # --------------------------------------------------------------------

        result = cv2.bitwise_and(frame, frame, mask=mask)

        ys, xs = np.nonzero(mask)          # column (x) positions where mask == 255

        if xs.size:                        # at least one bright pixel?
            avg_x = xs.mean()              # arithmetic mean, in pixel units
            avg_x_norm = (avg_x / (IMG_W-1) * 2) -1
            self.button_pub.publish(Float32(data=float(avg_x_norm)))

def main(args=None):
    rclpy.init(args=args)
    node = DetectButton()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()