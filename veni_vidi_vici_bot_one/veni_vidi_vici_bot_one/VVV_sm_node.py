import rclpy

import csv
import os

from enum import Enum

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist, PointStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Empty, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray

from nav2_simple_commander.robot_navigator import BasicNavigator

from ament_index_python.packages import get_package_share_directory

import numpy as np
import math
import time

from veni_vidi_vici_bot_one.VVV_util import *

class RobotState(Enum):

    S0_BOOT  = 0
    SALL_READY = 1

    S1_MOVE_TO_P10 = 2
    S1_PRESS_BUTTON = 3
    S1_MOVE_TO_P11 = 4

    SALL_SEARCH_FOR_DUPLO = 5
    SALL_MOVE_TO_DUPLO = 6
    SALL_MOVE_TO_CLOSEST_CORNER = 7
    SALL_RETURN_TO_S = 8
    SALL_DELOAD = 9

    S2_MOVE_TO_P20 = 10
    S2_MOVE_TO_P21 = 11
    S2_ALIGN_ROBOT = 12

    S3 = 13

class StateMachineNode(Node):

    def __init__(self):

        super().__init__('state_machine_node')

        self.get_logger().info('VVV State Machine node successfully started')

        self._priority_callback_group = MutuallyExclusiveCallbackGroup()
        self._default_callback_group  = MutuallyExclusiveCallbackGroup()

        self._init_publishers_and_subscribers()
        self._init_timers()    
        self._init_navigator()

        #----- Initialize state machine -----

        self.state = RobotState.S0_BOOT
        self.current_mission = 1
        self.start = False
        self.stop  = False

        self.start_time = None
        self.elapsed_time = 0.0
        self.started_moving = False

        #----- Hardcoded values from .csv -----

        self.initial_pose = None

        self.button_ = {}

        self.list_of_timeouts = []

        self.list_of_corners_Z1 = []
        self.list_of_corners_Z2 = []
        self.list_of_corners_Z3 = []

        #----- Updated values -----

        #----- Robot state in the world -----

        self.yaw = 0.0
        self.trans = None
        self.costmap = None

        self.front_distance = None
        self.left_distance  = None

        #----- Mission 1 related state -----

        self.wait_for_goal = False
        self.num_pass_narrow = 0.0

        self.S1_door_open = False
        self.num_press_attempt = 0.0

        self.adjust_orientation = True
        self.approach_button = False
        self.verify_door_is_open = False
        self.go_back_button = False
        self.wait_scan_counter = 0.0

        self.turn_left = False
        self.turn_right = False
        self.turn_mid = False
        self.des_theta_left = None
        self.des_theta_right = None
        self.des_theta_mid = None
        self.estimating_duplo_position = False
        self.duplo_found = False
        self.next_duplo = {}

        self.reached_duplo = False

        self.corner_x = None
        self.corner_y = None
        self.corner_theta = None
        self.corner_idx = None
        self.corner_reached = False
        self.mission1_available_corners = None
        self.mission2_available_corners = None
        self.mission3_available_corners = None

        self.number_of_duplos_collected = 0.0

        #----- Init state machine and the listener -----

        self.initialized = self._init_state_machine()

        self._init_tf_listener()
        self.get_logger().info('Done initializing !!')

    def destroyNode(self):

        super().destroy_node()

    #----- Init functions for all the sensing -----

    def _init_publishers_and_subscribers(self):

        #----- Create Publishers -----

        self.vel_pub       = self.create_publisher(Twist, '/cmd_vel_mn', 10)
        self.door_pub      = self.create_publisher(Float64MultiArray, '/door_servo/commands', 10)
        self.storage_pub   = self.create_publisher(Float64MultiArray, '/storage_servo/commands', 10)
        self.collector_pub = self.create_publisher(Float64MultiArray, '/collector/commands', 10)

        #----- Create Subscribers -----

        self.start_sub = self.create_subscription(Empty,    '/start_signal', self.start_callback, 1 , callback_group=self._priority_callback_group)

        self.imu_sub  = self.create_subscription(Imu,       '/imu/data', self.imu_callback, 1       , callback_group=self._default_callback_group)
        self.odom_sub = self.create_subscription(Odometry,  '/diff_cont/odom', self.odom_callback, 1, callback_group=self._default_callback_group)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1          , callback_group=self._default_callback_group)

        self.stop_sub = self.create_subscription(Empty,     '/stop_signal', self.stop_callback, 1   , callback_group=self._priority_callback_group)

        #self.yolov6_sub = self.create_subscription(SpatialDetectionArray, '/color/yolov6_Spatial_detections', self.yolov6_callback, 10, callback_group=self._priority_callback_group)

    def _init_timers(self):

        #----- Timer to execute SM -----

        self.timer_sm      = self.create_timer(0.2, self.step, callback_group=self._default_callback_group)
        
        #----- Update cost map at a slower rate -----

        self.timer_costmap = self.create_timer(3.0, self.timer_get_costmap_callback, callback_group=self._default_callback_group)

    def _init_tf_listener(self):

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_tf = self.create_timer(0.2, self.timer_tf_callback, callback_group=self._default_callback_group)

    def _init_navigator(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        #----- Load initial pose from csv -----

        try:
            initial_pose_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'initial_pose.csv'
            )     

            with open(initial_pose_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)
                row = next(reader)  # Only one row expected

                initial_x = float(row['x'])
                initial_y = float(row['y'])
                initial_theta = float(row['theta'])

        except Exception as e:

            self.get_logger().info("FAILED TO LOAD INITIAL POSE")

            initial_x = 0.0
            initial_y = 0.0
            initial_theta = 0.0

        #----- Set initial pose for AMCL to work -----

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg() #self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_x
        initial_pose.pose.position.y = initial_y
        initial_pose.pose.orientation.z = math.sin(initial_theta / 2.0)
        initial_pose.pose.orientation.w = math.cos(initial_theta / 2.0) 

        self.navigator.setInitialPose(initial_pose)

        self.initial_pose = initial_pose

    #----- STATE MACHINE RELATED FUNCTIONS -----

    def _init_state_machine(self):

        #----- Load the timeout thrs -----

        try:
            timeouts_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'list_of_timeouts.csv'
            )     

            with open(timeouts_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)
                row = next(reader)  # Only one row expected

                t1 = float(row['t1'])
                t2 = float(row['t2'])
                t3 = float(row['t3'])

            self.get_logger().info("SUCCESFULLY LOADED TIMEOUTS") 

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD TIMEOUTS")  

            t1 = 60.0
            t2 = 120.0
            t3 = 240.0  

        self.list_of_timeouts.append(t1)
        self.list_of_timeouts.append(t2)
        self.list_of_timeouts.append(t3)

        #----- Load Zone 1 corner points -----

        try:
            zone1_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'mission1_points_pose.csv'
            )     

            with open(zone1_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)

                for row in reader:

                    corner_point = {}

                    corner_point['x']     = float(row['x'])
                    corner_point['y']     = float(row['y'])
                    corner_point['theta'] = float(row['theta'])

                    self.list_of_corners_Z1.append(corner_point)

            self.get_logger().info("SUCCESFULLY LOADED ZONE 1 POINTS") 

            self.mission1_available_corners = [self.list_of_corners_Z1[2], self.list_of_corners_Z1[3], self.list_of_corners_Z1[4]]

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD ZONE 1 POINTS")  

        #----- Load Zone 2 corner points -----

        try:
            zone2_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'mission2_points_pose.csv'
            )     

            with open(zone2_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)

                for row in reader:

                    corner_point = {}

                    corner_point['x']     = float(row['x'])
                    corner_point['y']     = float(row['y'])
                    corner_point['theta'] = float(row['theta'])

                    self.list_of_corners_Z2.append(corner_point)

            self.get_logger().info("SUCCESFULLY LOADED ZONE 2 POINTS") 

            self.mission2_available_corners = [self.list_of_corners_Z2[2], self.list_of_corners_Z2[3], self.list_of_corners_Z2[4]]

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD ZONE 2 POINTS") 

        #----- Load Zone 3 corner points -----

        try:
            zone3_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'mission3_points_pose.csv'
            )     

            with open(zone3_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)

                for row in reader:

                    corner_point = {}

                    corner_point['x']     = float(row['x'])
                    corner_point['y']     = float(row['y'])
                    corner_point['theta'] = float(row['theta'])

                    self.list_of_corners_Z3.append(corner_point)

            self.get_logger().info("SUCCESFULLY LOADED ZONE 3 POINTS") 

            self.mission3_available_corners = self.list_of_corners_Z3.copy()
            del self.mission3_available_corners[0]

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD ZONE 3 POINTS") 

        #----- Load Button press pose -----

        try:
            button_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'button_pose.csv'
            )     

            with open(button_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)
                row = next(reader)  # Only one row expected

                self.button_['x'] = float(row['x'])
                self.button_['y'] = float(row['y'])
                self.button_['theta'] = float(row['theta'])
                self.button_['door_range'] = float(row['door_range'])

            self.get_logger().info("SUCCESFULLY LOADED BUTTON PRESS POSE") 

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD BUTTON PRESS POSE") 

        return len(self.list_of_timeouts) == 3 and len(self.list_of_corners_Z1) == 5 and len(self.list_of_corners_Z2) == 5 and len(self.list_of_corners_Z3) == 5
    
    def step(self):

        if   self.state == RobotState.S0_BOOT:

            next_state = self.execute_S0_BOOT()

        elif self.state == RobotState.SALL_READY: 

            next_state = self.execute_SALL_READY()

        elif self.state == RobotState.S1_MOVE_TO_P10: 

            next_state = self.execute_S1_MOVE_TO_P10()

        elif self.state == RobotState.S1_PRESS_BUTTON:

            next_state = self.execute_S1_PRESS_BUTTON()

        elif self.state == RobotState.S1_MOVE_TO_P11:

            next_state = self.execute_S1_MOVE_TO_P11()

        self.state = next_state

        #----- Update elapsed time -----

        if self.started_moving:

            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time

            self.elapsed_time = elapsed_time.nanoseconds * 1e-9

    #----- Define all the callbacks -----

    def timer_tf_callback(self):

        try:

            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            _, _, yaw = quaternion_to_euler(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)

            self.yaw = yaw
            self.trans = trans
        
        except TransformException as ex:

            self.get_logger().info(f'Could not transform base_link to map: {ex}')

    def timer_get_costmap_callback(self):

        self.costmap = self.navigator.getGlobalCostmap()

    def imu_callback(self, msg):

        pitch, _, _ = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def odom_callback(self, msg):

        _,_,yaw = quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def scan_callback(self, msg):

        front_index = int((0 - msg.angle_min) / msg.angle_increment)
        self.front_distance = msg.ranges[front_index]

        left_index = int((1.57 - msg.angle_min) / msg.angle_increment)
        self.left_distance = msg.ranges[left_index]

    def start_callback(self, msg):

        self.start = True
        self.stop  = False

    def stop_callback(self, msg):

        self.stop  = True
        self.start = False

        self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

 #   def yolov6_callback(self, msg):

 #       return

    #----- Define all the publisher functions -----

    def _publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.vel_pub.publish(msg)

    def _publish_door_servo_cmd(self, servo_command=[0.0]):

        msg = Float64MultiArray()
        msg.data = servo_command

        self.door_pub.publish(msg)

    def _publish_storage_servo_cmd(self, servo_command=[0.0]):

        msg = Float64MultiArray()
        msg.data = servo_command

        self.storage_pub.publish(msg)

    def _publish_collector_cmd(self, collector_command=[0.0]):

        msg = Float64MultiArray()
        msg.data = collector_command

        self.collector_pub.publish(msg)