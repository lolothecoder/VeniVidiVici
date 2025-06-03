# 
#!/usr/bin/env python3
"""
detect_ball.py  –  YOLOv11 version

• Sub : /image_in      (sensor_msgs/Image)
• Pub : /detected_ball (geometry_msgs/Point)  x=u_norm, y=v_norm, z=size_norm
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node          import Node
from sensor_msgs.msg     import Image
from geometry_msgs.msg   import Point
from cv_bridge           import CvBridge
from ultralytics import YOLO

# image resolution (px)
IMG_W, IMG_H = 640, 480

class DetectBall(Node):
    def __init__(self):
        super().__init__('detect_ball_yolo')

        self.bridge = CvBridge()
        self.yolo   = YOLO('/site_config/best_ncnn_model1', task='detect')  
        # or use a built‐in model: YOLO('yolov8n.pt'), but you need custom data

        self.image_sub = self.create_subscription(
            Image, '/image_in', self.callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        self.ball_pub = self.create_publisher(Point, '/detected_ball', 1)

        self.get_logger().info('YOLOv11 duplos detector initialized.')

    def callback(self, msg: Image):
        # 1. convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 2. run YOLOv11 inference
        #    returns a list of Results; we take the first (and only) batch
        results = self.yolo(frame, conf=0.6, stream=False)[0]

        best_pt = Point()             # x,y normalized; z holds normalized size
        best_conf = 0.0

        # 3. loop detections
        for box in results.boxes:
            cls   = int(box.cls[0])   # class index
            conf  = float(box.conf[0])
            if cls != 0:              # assume class 0 = tennis‐ball
                continue

            # bounding‐box coords
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            w = x2 - x1
            h = y2 - y1

            # center in pixel
            cx = x1 + w/2
            cy = y1 + h/2

            # normalize to [-1,1]
            u_norm =  2 * (cx/(IMG_W-1)) - 1
            v_norm =  2 * (cy/(IMG_H-1)) - 1
            size_norm = (w*h)/(IMG_W*IMG_H)  # area fraction

            if conf > best_conf:
                best_conf = conf
                best_pt.x = u_norm
                best_pt.y = v_norm
                best_pt.z = size_norm

        # 4. publish the best detection (if any)
        if best_conf > 0:
            self.ball_pub.publish(best_pt)
            self.get_logger().debug(
                f"Det ball: u={best_pt.x:.3f}, v={best_pt.y:.3f}, size={best_pt.z:.4f}, conf={best_conf:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = DetectBall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()