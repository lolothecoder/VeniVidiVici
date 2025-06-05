#!/usr/bin/env python3
"""
detect_duplo_3d.py
────────────────────────
• Sub  : /detected_duplo      (geometry_msgs/Point)  x = u_norm, y = v_norm
• Pub  : /detected_duplo_3d     (geometry_msgs/Point)  frame = camera_link_optical
         /ball_3d_marker       (visualization_msgs/Marker)

Uses TF to obtain the live transform map → camera_link_optical.
"""

import math
import numpy as np
import cv2
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_matrix

# ────────── camera intrinsics ───────────────────────────────────────────
K = np.array([[1133.10504192, 0, 628.21074810],
              [0, 1133.44274070, 375.68857455],
              [0,       0,   1]], dtype=np.float64)
K_inv = np.linalg.inv(K)

IMG_W, IMG_H = 640, 480                                 # pixels
CENTER_THRESH_X = 0.99                                     # normalised pixels 
CENTER_THRESH_Y = 0.99                                     # normalised pixels

# ────────── ROS2 node ───────────────────────────────────────────────────
class DetectBall3D(Node):
    def __init__(self):
        super().__init__('detect_duplo_3d')

        # TF buffer / listener
        self.tfbuf  = tf2_ros.Buffer()
        self.tflist = tf2_ros.TransformListener(self.tfbuf, self,
                                                spin_thread=True)

        self.create_subscription(Point, '/detected_duplo', self.cb, 10)
        self.pub  = self.create_publisher(Point,  '/detected_duplo_3d', 10)
        self.mpub = self.create_publisher(Marker, '/ball_3d_marker', 10)

        self.ball_radius = 0.033
        self.get_logger().info('Node initialised (TF version).')

    # ------------------------------------------------------------------
    def cb(self, msg: Point):
        if abs(msg.x) > CENTER_THRESH_X or abs(msg.y) > CENTER_THRESH_Y:
            return

        # 1. normalised (-1…+1) → pixel (0…W-1 / 0…H-1)
        u_px = (msg.x + 1.0) * 0.5 * (IMG_W - 1)
        v_px = (msg.y + 1.0) * 0.5 * (IMG_H - 1)

        # 2. build image ray in camera frame (undistort not needed in sim)
        ray_cam = K_inv @ np.array([u_px, v_px, 1.0], np.float64)

        # # 3. get map → camera_link_optical transform
        if not self.tfbuf.can_transform('camera_link_optical', 'map',
                                        rclpy.time.Time()):
            return                                           # TF not ready
        try:
            tr = self.tfbuf.lookup_transform('camera_link_optical', 'map',
                                              rclpy.time.Time(),
                                              timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'TF unavailable: {e}')
            return

        # ▸ rotation R_mc  (map → cam)  and translation t_mc (map-origin in cam)
        R_mc = quaternion_matrix([tr.transform.rotation.x,
                                  tr.transform.rotation.y,
                                  tr.transform.rotation.z,
                                  tr.transform.rotation.w])[:3, :3]
        t_mc = np.array([[tr.transform.translation.x],
                         [tr.transform.translation.y],
                         [tr.transform.translation.z]])

        # 4. ground-plane (z=0 in map) expressed in camera frame
        n_c = R_mc @ np.array([0, 0, 1], np.float64)       # plane normal
        d_c = -n_c @ t_mc.ravel()                          # offset

        denom = n_c @ ray_cam
        if abs(denom) < 1e-6:                              # horizon/sky
            return
        t = -d_c / denom
        if t <= 0:
            return
        P_cam = t * ray_cam                                # hit-point cam-frame

        # 5. convert to map frame for debugging (inverse transform)
        P_map = R_mc.T @ (P_cam.reshape(3, 1) - t_mc)
        P_map = P_map.ravel()

        # 6. publish point and marker
        pt = Point(x=float(P_cam[0]), y=float(P_cam[1]), z=float(P_cam[2]))
        self.pub.publish(pt)

        mk = Marker()
        mk.header.frame_id = 'camera_link_optical'
        mk.type, mk.action = mk.SPHERE, mk.ADD
        mk.pose.position   = pt
        mk.scale.x = mk.scale.y = mk.scale.z = self.ball_radius * 2
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.0, 0.0, 1.0
        self.mpub.publish(mk)

        # 7. debug prints
        dist_cam = np.linalg.norm(P_cam)
        # self.get_logger().info(
        #     f"P_cam {P_cam[0]:6.2f} {P_cam[1]:6.2f} {P_cam[2]:6.2f}  "
        #     f"| dist {dist_cam:5.2f} m"
        # )
        # self.get_logger().info(
        #     f"P_map {P_map[0]:6.2f} {P_map[1]:6.2f} {P_map[2]:6.2f}"
        # )

# ────────── main ────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DetectBall3D())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
