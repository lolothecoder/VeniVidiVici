import rclpy

import csv
import os

from enum import Enum

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Empty, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray

from nav2_simple_commander.robot_navigator import BasicNavigator

from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


import numpy as np
import math

from veni_vidi_vici_bot_one.VVV_util import *

class RobotState(Enum):

    S0_BOOT  = 0
    SALL_READY = 1

    S1_MOVE_TO_P10 = 2
    S1_PRESS_BUTTON = 3
    S1_MOVE_TO_P11 = 4
    S1_EXIT_ROOM = 5

    SALL_SEARCH_FOR_DUPLO = 6
    SALL_MOVE_TO_DUPLO = 7
    SALL_MOVE_TO_CLOSEST_CORNER = 8
    SALL_RETURN_TO_S = 9
    SALL_DELOAD = 10

    S3_MOVE_TO_P30 = 11
    S3_MOVE_TO_P31 = 12
    S3_ALIGN_ROBOT = 13
    S3_EXIT_RAMP   = 14

    S_STOP = 15

class StateMachineNode(Node):

    def __init__(self):

        super().__init__('state_machine_node')

        self.get_logger().info('VVV State Machine node successfully started')

        self._priority_callback_group = MutuallyExclusiveCallbackGroup()
        self._default_callback_group  = MutuallyExclusiveCallbackGroup()

        #----- Initialize state machine -----

        self.state = RobotState.S0_BOOT
        self.current_mission = 1
        self.start = False
        self.stop  = False

        self.N_mission1_executed = 0
        self.N_mission2_executed = 0
        self.N_mission3_executed = 0

        self.start_time = None
        self.elapsed_time = 0.0
        self.started_moving = False

        self.running_timeout1 = False

        #----- Hardcoded values from .csv -----

        self.S_pose = {}

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
        self.right_distance  = None

        self.list_of_visited_duplos = []

        self.block_record = False

        #----- Mission 1 related state -----

        self.align_with_duplo = False
        self.go_straight_to_duplo = False
        self.nav2_to_duplo        = False


        self.wait_for_goal = False
        self.manual_recovery = False
        self.adjust_position = False
        self.adjust_orientation = False
        self.previous_linear_x_mn = 0.0

        self.number_of_recoveries = 0

        self.prepare_press_button = True
        self.button_attempts = 0.0
        self.prepare_enter = True
        self.align_enter = False
        self.go_through_door = False

        self.S1_door_open = False
        self.num_press_attempt = 0.0
        self.prepare_press_button = True
        self.des_x_button = None
        self.des_y_button = None
        self.des_theta_button = None
        self.door_thr_button = None
        self.align_button = False
        self.approach_button = False
        self.finished_action_time = None
        self.wait_for_door_to_open = False
        self.verify_door_is_open = False
        self.go_back_button = False
        self.des_x_ret = None
        self.des_y_ret = None
        self.match_door_orientation = False
        self.wait_scan_counter = 0.0

        self.prepared_exit = False
        self.prepare_x = None
        self.prepare_y = None
        self.prepare_theta = None
        self.align_exit = False
        self.match_exit_orientation = False
        self.exit_x = None
        self.exit_y = None
        self.exit_theta = None

        self.turn_left = False
        self.turn_right = False
        self.turn_mid = False
        self.des_theta_left = None
        self.des_theta_right = None
        self.des_theta_mid = None
        self.estimating_duplo_position = False
        self.duplo_found = False
        self.next_duplo = {}
        self._best_dist = 10000

        self.current_duplo_x = None
        self.current_duplo_y = None
        self.current_duplo_theta = None
        self.reached_duplo = False

        self.choose_corner = True
        self.corner_x = None
        self.corner_y = None
        self.corner_theta = None
        self.goal_theta = None
        self.corner_idx = None
        self.align_corner = False
        self.go_straight_to_corner = False
        self.nav2_to_corner = False

        self.mission1_available_corners = None
        self.mission2_available_corners = None
        self.mission3_available_corners = None
        self.match_corner_orientation = False

        self.number_of_duplos_collected = 0.0
        self.duplos_capacity = 5

        self.start_deload = True

        #----- Temporary -----

        self.list_of_duplos_mission1 = []
        self.list_of_duplos_mission2 = []
        self.list_of_duplos_mission3 = []

        #----- Initialize everything -----

        self._init_publishers_and_subscribers()
        self._init_timers()    
        self._init_navigator()

        #----- Connect to srv for cleaning costmap -----

        self.get_logger().info('Costmap init !!')
        self._init_costmap()
        self.get_logger().info('Done costmap init !!')

        #----- State machine -----

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
        self.ramp_pub      = self.create_publisher(Float64MultiArray, '/ramp_servo/commands', 10)
        self.collector_pub = self.create_publisher(Float64MultiArray, '/collector_servo/commands', 10)

        #----- Create Subscribers -----

        self.start_sub = self.create_subscription(Empty,    '/start_signal', self.start_callback, 1 , callback_group=self._priority_callback_group)

        self.odom_sub = self.create_subscription(Odometry,  '/diff_cont/odom', self.odom_callback, 1, callback_group=self._default_callback_group)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1          , callback_group=self._default_callback_group)
        #self.yolov11_sub = self.create_subscription(Point,  '/detected_ball_3d_map', self.yolov11_callback, 10, callback_group=self._default_callback_group)

        self.stop_sub = self.create_subscription(Empty,     '/stop_signal', self.stop_callback, 1   , callback_group=self._priority_callback_group)

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

    def _init_costmap(self):

        self.local_costmap_client  = self.create_client(SetParameters, '/local_costmap/set_parameters')
        self.global_costmap_client = self.create_client(SetParameters, '/global_costmap/set_parameters')

    #----- STATE MACHINE RELATED FUNCTIONS -----

    def _init_state_machine(self):

        #----- Load S pose -----

        try:
            s_pose_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'S_pose.csv'
            )     

            with open(s_pose_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)
                row = next(reader)  # Only one row expected

                s_x = float(row['x'])
                s_y = float(row['y'])
                s_theta = float(row['theta'])

            self.get_logger().info("SUCCESFULLY LOADED S POSE") 

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD S POSE")  

            s_x = 0.0
            s_y = 0.0
            s_theta = -np.pi/4  

        self.S_pose['x'] = s_x
        self.S_pose['y'] = s_y
        self.S_pose['theta'] = s_theta

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

            t1 = 210.0
            t2 = 390.0
            t3 = 540.0  

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

    def _mission1_reset_params(self):

        self.wait_for_goal = False
        self.manual_recovery = False
        self.previous_linear_x_mn = 0.0

        self.number_of_recoveries = 0

        self.mission1_abandoned = False
        self.S1_door_open = False
        self.num_press_attempt = 0.0
        self.align_button = True
        self.approach_button = False
        self.verify_door_is_open = False
        self.go_back_button = False
        self.wait_scan_counter = 0.0
        self.prepared_exit = False
        self.prepare_x = None
        self.prepare_y = None
        self.prepare_theta = None
        self.align_exit = False
        self.match_exit_orientation = False
        self.exit_x = None
        self.exit_y = None
        self.exit_theta = None

        self.adjust_orientation = False

        self.turn_left = False
        self.turn_right = False
        self.turn_mid = False
        self.des_theta_left = None
        self.des_theta_right = None
        self.des_theta_mid = None
        self.estimating_duplo_position = False
        self.duplo_found = False
        self.next_duplo = {}

        self.current_duplo_x = None
        self.current_duplo_y = None
        self.current_duplo_theta = None
        self.reached_duplo = False

        self.corner_x = None
        self.corner_y = None
        self.corner_theta = None
        self.corner_idx = None
        self.corner_reached = False
        self.match_corner_orientation = False
    
    def disable_costmaps(self):

        self.set_costmap_enabled(self.local_costmap_client, False)
        self.set_costmap_enabled(self.global_costmap_client, False)

    def enable_costmaps(self):

        self.set_costmap_enabled(self.local_costmap_client, True)
        self.set_costmap_enabled(self.global_costmap_client, True)

    def set_costmap_enabled(self, client, enabled: bool):

        param = Parameter(name='obstacle_layer.enabled', value=enabled)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]
        future = client.call_async(req)

    def step(self):

        if   self.state == RobotState.S0_BOOT:

            next_state = self.execute_S0_BOOT()

        elif self.state == RobotState.SALL_READY: 

            next_state = self.execute_SALL_READY()

        elif self.state == RobotState.S1_MOVE_TO_P10: 

            next_state = self.execute_S1_MOVE_TO_P10()
            
        elif self.state == RobotState.S1_PRESS_BUTTON:

            next_state = self.execute_S1_PRESS_BUTTON()

        else:

            next_state = RobotState.S_STOP

        #----- ADD NEW STATES -----

        #----- Update elapsed time -----

        if self.started_moving:

            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time

            self.elapsed_time = elapsed_time.nanoseconds * 1e-9

        self.state = next_state

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

    def odom_callback(self, msg):

        _,_,yaw = quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def scan_callback(self, msg):

        front_index = int((0 - msg.angle_min) / msg.angle_increment)
        self.front_distance = msg.ranges[front_index]

        left_index = int((1.57 - msg.angle_min) / msg.angle_increment)
        self.left_distance = msg.ranges[left_index]

        right_index = int((-1.57 - msg.angle_min) / msg.angle_increment)
        self.right_distance = msg.ranges[right_index]

    def start_callback(self, msg):

        self.start = True
        self.stop  = False

    def stop_callback(self, msg):

        self.stop  = True
        self.start = False

        self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

    # def yolov11_callback(self, msg: Point):

    #     if self.state in [RobotState.SALL_SEARCH_FOR_DUPLO, RobotState.SALL_MOVE_TO_CLOSEST_CORNER, RobotState.S1_MOVE_TO_P11] and not self.block_record:  #, RobotState.S1_MOVE_TO_P11

    #         #----- If this detection is closer than any before and duplo not taken already, remember it

    #         dx, dy = msg.x, msg.y

    #         not_already_visited = True

    #         for duplo in self.list_of_visited_duplos:

    #             dist_duplo = math.hypot(dx - duplo['x'], dy - duplo['y'])
    #             if dist_duplo < 0.5: 
                    
    #                 not_already_visited = False
    #                 break
            
    #         if not_already_visited:

    #             rx = self.trans.transform.translation.x
    #             ry = self.trans.transform.translation.y

    #             dist = math.hypot(dx - rx, dy - ry)
            
    #             if dist < self._best_dist:

    #                 self._best_dist      = dist
    #                 self.nearest_duplo_x = dx
    #                 self.nearest_duplo_y = dy

    #     else:

    #             self._best_dist      = 10000
    #             self.nearest_duplo_x = None
    #             self.nearest_duplo_y = None           

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

    def _publish_ramp_servo_cmd(self, servo_command=[0.0]):

        msg = Float64MultiArray()
        msg.data = servo_command

        self.ramp_pub.publish(msg)

    def _publish_collector_servo_cmd(self, collector_command=[0.0]):

        msg = Float64MultiArray()
        msg.data = collector_command

        self.collector_pub.publish(msg)