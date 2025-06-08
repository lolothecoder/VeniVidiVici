#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class GoToPosition(Node):
    def __init__(self):
        super().__init__('go_to_position')
        # Target offset (in meters) from start pose
        self.target_x = 0.0
        self.target_y = 1.0
        # Movement parameters
        self.speed = 0.2      # m/s
        self.tolerance = 0.01 # meters

        # Internal state
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        # Publisher for velocity commands on cmd_vel_unstamped
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_unstamped', 10)
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Waiting for first odometry message...')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            goal_x = self.start_x + self.target_x
            goal_y = self.start_y + self.target_y
            self.get_logger().info(
                f'Start pose: x={self.start_x:.3f}, y={self.start_y:.3f}\n'
                f'Moving towards: x={goal_x:.3f}, y={goal_y:.3f}'
            )
        self.current_x = x
        self.current_y = y

    def control_loop(self):
        # Wait until we have odometry
        if self.start_x is None or self.current_x is None:
            return

        # Absolute goal position
        goal_x = self.start_x + self.target_x
        goal_y = self.start_y + self.target_y

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        dist = math.hypot(dx, dy)

        twist = Twist()
        if dist > self.tolerance:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Reached target position, stopping.')
            self.timer.cancel()
            rclpy.shutdown()

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        # Ensure stop on shutdown
        try:
            stop = Twist()
            self.cmd_pub.publish(stop)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoToPosition()
    # Handle Ctrl-C cleanly
    signal.signal(signal.SIGINT, lambda sig, frame: rclpy.shutdown())
    rclpy.spin(node)
    node.destroy_node()
    # No extra shutdown


if __name__ == '__main__':
    main()
