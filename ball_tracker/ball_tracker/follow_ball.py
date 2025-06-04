import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from tf_transformations import quaternion_matrix


class FollowDuplo3D(Node):
    def __init__(self):
        super().__init__("follow_duplo_3d")

        # ── PARAMETERS ──────────────────────────────────────────────────────
        # Timeout: how many seconds before we consider the Duplo "lost"
        self.declare_parameter("rcv_timeout_secs", 1.0)
        # Proportional gains for linear (forward) and angular (yaw) velocity
        self.declare_parameter("k_linear", 0.5)
        self.declare_parameter("k_angular", 1.0)
        # Desired stopping distance (m) between robot front (base_link origin) and Duplo
        self.declare_parameter("desired_distance", 0.5)
        # Maximum allowed speeds
        self.declare_parameter("max_lin_speed", 0.2)
        self.declare_parameter("max_ang_speed", 0.5)
        # Low‐pass filter coefficient (0.0 … 1.0) for incoming 3D point
        self.declare_parameter("filter_value", 0.9)

        # Retrieve parameter values
        self.rcv_timeout_secs = (
            self.get_parameter("rcv_timeout_secs")
            .get_parameter_value()
            .double_value
        )
        self.k_linear = (
            self.get_parameter("k_linear").get_parameter_value().double_value
        )
        self.k_angular = (
            self.get_parameter("k_angular").get_parameter_value().double_value
        )
        self.desired_distance = (
            self.get_parameter("desired_distance").get_parameter_value().double_value
        )
        self.max_lin_speed = (
            self.get_parameter("max_lin_speed").get_parameter_value().double_value
        )
        self.max_ang_speed = (
            self.get_parameter("max_ang_speed").get_parameter_value().double_value
        )
        self.filter_value = (
            self.get_parameter("filter_value").get_parameter_value().double_value
        )

        # ── TF2 SETUP ──────────────────────────────────────────────────────
        # We will look up transforms from 'camera_link_optical' → 'base_link'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # ── SUBSCRIBER ────────────────────────────────────────────────────
        # Expects a geometry_msgs/Point in frame 'camera_link_optical'
        self.create_subscription(Point, "/detected_duplo_3d", self._duplo_callback, 10)

        # ── PUBLISHER ─────────────────────────────────────────────────────
        # We publish velocity commands to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── INTERNAL STATE ─────────────────────────────────────────────────
        # Filtered 3D position of Duplo (in camera frame)
        self.filtered_pt_cam = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        # Time when last valid detection was received
        self.last_rcv_time = time.time() - 10.0  # initialize to "long ago"

        # ── TIMER: MAIN CONTROL LOOP ────────────────────────────────────────
        timer_period = 0.1  # seconds (10 Hz)
        self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info("FollowDuplo3D node started.")

    def _duplo_callback(self, msg: Point):
        """
        Incoming 3D detection callback.
        msg.x, msg.y, msg.z are coordinates of the Duplo in camera_link_optical frame.
        Apply a simple low‐pass filter, and update the timestamp.
        """
        f = self.filter_value
        raw = np.array([msg.x, msg.y, msg.z], dtype=np.float64)
        self.filtered_pt_cam = raw
        # Exponential moving average
        # self.filtered_pt_cam = f * self.filtered_pt_cam + (1.0 - f) * raw
        self.last_rcv_time = time.time()

    def _timer_callback(self):
        """
        Runs at ~10 Hz. If we have a recent Duplo detection, transform the filtered
        camera‐frame point into base_link, compute control commands, and publish.
        Otherwise, stop the robot.
        """
        now = time.time()
        dt = now - self.last_rcv_time

        twist = Twist()

        # If data is too old, consider the Duplo "lost" and zero the velocities
        if dt > self.rcv_timeout_secs:
            self.get_logger().warn("Duplo lost (no recent detection); stopping.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Otherwise, we have a fresh filtered position in camera frame:
        pt_cam = self.filtered_pt_cam.copy()

        # ── LOOK UP TRANSFORM ───────────────────────────────────────────
        # We want the transform: camera_link_optical → base_link
        try:
            # target_frame='base_link', source_frame='camera_link_optical'
            t = self.tf_buffer.lookup_transform(
                "base_link", "camera_link_optical", rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}. Stopping.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Extract rotation (as 4×4 matrix) and translation (3×1) from the TransformStamped
        quat = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ]
        T_cam_to_base = quaternion_matrix(quat)  # 4×4 homogeneous (rotation + zeros)
        T_cam_to_base[0, 3] = t.transform.translation.x
        T_cam_to_base[1, 3] = t.transform.translation.y
        T_cam_to_base[2, 3] = t.transform.translation.z

        # Build homogeneous vector for the filtered 3D point
        p_cam_hom = np.array([pt_cam[0], pt_cam[1], pt_cam[2], 1.0], dtype=np.float64)
        # Transform into base_link frame
        p_base_hom = T_cam_to_base @ p_cam_hom
        # Extract x, y, z in base_link
        bx, by, bz, _ = p_base_hom

        # ── CONTROL LAW ─────────────────────────────────────────────────
        # 1) Compute planar "forward" error: how far along x‐axis of base_link 
        #    minus desired_distance.
        forward_error = bx - self.desired_distance

        # 2) Compute yaw error: angle between robot's forward axis and the Duplo's location
        #    This is atan2(y, x) in the base_link frame.
        ang_error = math.atan2(by, bx)

        # 3) Proportional control
        v_cmd = self.k_linear * forward_error
        w_cmd = self.k_angular * ang_error

        # 4) Clip to max speeds
        v_cmd = max(-self.max_lin_speed, min(self.max_lin_speed, v_cmd))
        w_cmd = max(-self.max_ang_speed, min(self.max_ang_speed, w_cmd))

        # 5) If Duplo is extremely close (forward_error < some small threshold), stop forward motion
        if bx < 0.1:
            v_cmd = 0.0

        # 6) Package into Twist
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)

        # 7) Logging (optional, can be commented out later)
        self.get_logger().info(
            f"Duplo (base_frame): x={bx:.2f} m, y={by:.2f} m, z={bz:.2f} m | "
            f"err_forward={forward_error:.2f}, err_yaw={ang_error:.2f} → "
            f"v={v_cmd:.2f}, ω={w_cmd:.2f}"
        )

        # 8) Publish velocity command
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowDuplo3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops before shutdown
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()