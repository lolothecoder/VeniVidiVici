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
from std_msgs.msg import Float64MultiArray, Float32

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
    S1_TRANSIT_STATE = 6

    SALL_SEARCH_FOR_DUPLO = 7
    SALL_MOVE_TO_DUPLO = 8
    SALL_MOVE_TO_CLOSEST_CORNER = 9
    SALL_RETURN_TO_S = 10
    SALL_DELOAD = 11

    S3_MOVE_TO_P30 = 12
    S3_MOVE_TO_P31 = 13
    S3_ALIGN_ROBOT = 14
    S3_EXIT_RAMP   = 15

    S_STOP = 16
    S1_SWEEP = 17

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
        self.running_timeout2 = False
        
        #----- Hardcoded values from .csv -----

        self.duplos_capacity = 20

        self.S_pose = {}
        self.T_pose = {}

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

        self.mission1_abandoned = False

        self.lidar_dist_counter = 0.0

        #----- Mission 1 related state -----

        self.list_of_mission1_hardcoded_points = [

            [11.3, 3.0, -np.pi/2],
            [11.3, 3.5, -np.pi/2],
            [11.3, 4.2,  np.pi],
            [10.5, 4.2,  np.pi],
            [9.7,  4,2,  -np.pi/2],
            [9.7,  3.5,  -np.pi/2],
            [9.7,  3.0,  0.0],
            [10.5, 3.0, 0.0],
            [11.0, 3.5, -np.pi/2],
            [10.5, 4.0, np.pi],
            [10.0, 3.5, -np.pi/2]
        ]
        self.first_time_sweep = True

        self.align_with_duplo = False
        self.go_straight_to_duplo = False
        self.nav2_to_duplo        = False

        self.button_normalized_coord = None
        self.align_wall = False

        self.wait_for_goal = False
        self.manual_recovery = False
        self.adjust_orientation = False
        self.previous_linear_x_mn = 0.0

        self.prepare_press_button = True
        self.button_attempts = 0.0

        self.S1_door_open = False
        self.align_button = False
        self.approach_button = False
        self.finished_action_time = None
        self.wait_for_door_to_open = False
        self.verify_door_is_open = False
        self.go_back_button = False

        self.turn_left = False
        self.turn_right = False
        self.turn_mid = False
        self.des_theta_left = None
        self.des_theta_right = None
        self.des_theta_mid = None
        self.estimating_duplo_position = False
        self.duplo_found = False
        self.next_duplo = {}
        self._best_area = -1.0

        self.current_duplo_x = None
        self.current_duplo_y = None
        self.current_duplo_theta = None
        self.nearest_duplo_x = None
        self.nearest_duplo_y = None
        self.move_to_duplo_more = False
        self.start_distance = None
        self.current_duplo_x_in_frame = None
        self.current_duplo_y_in_frame = None
        self.distance_to_duplo = None

        self.align_manual = False
        self.length = None

        self.mission1_available_corners = None
        self.mission2_available_corners = None
        self.mission3_available_corners = None

        self.number_of_duplos_collected = 0.0

        self.prepare_exit = True
        self.exit_protection = False
        self.rotate_exit = False
        self.approach_exit = False
        self.exit_door = False

        self.prepare_deload = True
        self.deload_move = False

        self.prepare_corner = True
        self.corner_x = None
        self.corner_y = None 
        self.corner_theta = None 
        self.corner_idx = None
        self.corner_goal_theta = None
        self.go_straight_to_corner = False
        self.nav2_to_corner = False
        self.align_corner = False
        self.match_corner_orientation = False

        #----- Mission 2 setpoints -----

        self.list_of_mission2_hardcoded_points = [

            [11.5, 4.2,  np.pi],
            [10.5, 4.2,  np.pi],
            [9.7,  4.2, -np.pi/2],
            [9.7,  3.5, -np.pi/2],
            [9.7,  3.0,  0.0],
            [10.5, 3.0,  0.0],
            [11.0, 3.5, -np.pi/2],
            [10.5, 4.0,  np.pi],
            [10.0, 3.5, -np.pi/2]
        ]
        self.first_time_sweep2 = True        

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

        self.button_image_run_pub = self.create_publisher(Float32, '/button_image_run', 10)

        #----- Create Subscribers -----

        self.start_sub = self.create_subscription(Empty,    '/start_signal', self.start_callback, 1 , callback_group=self._priority_callback_group)

        self.odom_sub = self.create_subscription(Odometry,  '/diff_cont/odom', self.odom_callback, 1, callback_group=self._default_callback_group)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1          , callback_group=self._default_callback_group)

        self.yolov11_sub = self.create_subscription(Point,  '/detected_duplo_3d', self.yolov11_callback, 10, callback_group=self._default_callback_group)
        self.yolov11_in_frame_sub = self.create_subscription(Point,  '/detected_duplo', self.yolov11_in_frame_callback, 10, callback_group=self._default_callback_group)
        self.button_pose_sub = self.create_subscription(Float32, '/button_pose', self.button_pose_callback, 1, callback_group=self._default_callback_group )

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

        #----- Load Transient pose -----

        try:
            s_pose_csv_path = os.path.join(
                get_package_share_directory('veni_vidi_vici_bot_one'),
                'general',
                'transient_pose.csv'
            )     

            with open(s_pose_csv_path, mode='r') as csv_file:

                reader = csv.DictReader(csv_file)
                row = next(reader)  # Only one row expected

                t_x = float(row['x'])
                t_y = float(row['y'])
                t_theta = float(row['theta'])

            self.get_logger().info("SUCCESFULLY LOADED TRANSIENT POSE") 

        except Exception as e:   

            self.get_logger().info("FAILED TO LOAD TRANSIENT POSE")  

            t_x = 4.5
            t_y = 4.5
            t_theta = -np.pi/4  

        self.T_pose['x'] = t_x
        self.T_pose['y'] = t_y
        self.T_pose['theta'] = t_theta

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

            self.mission1_available_corners = self.list_of_corners_Z1.copy()
            del self.mission1_available_corners[0]
            del self.mission1_available_corners[0]

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

            self.mission2_available_corners = self.list_of_corners_Z2.copy()
            del self.mission2_available_corners[0]

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

        return len(self.list_of_timeouts) == 3 and len(self.list_of_corners_Z1) == 8 and len(self.list_of_corners_Z2) == 8 and len(self.list_of_corners_Z3) == 5

    def _mission_reset_params(self):

        self.align_with_duplo = False
        self.go_straight_to_duplo = False
        self.nav2_to_duplo        = False

        self.button_normalized_coord = None
        self.align_wall = False

        self.wait_for_goal = False
        self.manual_recovery = False
        self.adjust_orientation = False
        self.previous_linear_x_mn = 0.0

        self.prepare_press_button = True
        self.button_attempts = 0.0

        self.S1_door_open = False
        self.align_button = False
        self.approach_button = False
        self.finished_action_time = None
        self.wait_for_door_to_open = False
        self.verify_door_is_open = False
        self.go_back_button = False

        self.turn_left = False
        self.turn_right = False
        self.turn_mid = False
        self.des_theta_left = None
        self.des_theta_right = None
        self.des_theta_mid = None
        self.estimating_duplo_position = False
        self.duplo_found = False
        self.next_duplo = {}
        self._best_area = -1.0

        self.current_duplo_x = None
        self.current_duplo_y = None
        self.current_duplo_theta = None
        self.nearest_duplo_x = None
        self.nearest_duplo_y = None
        self.move_to_duplo_more = False
        self.start_distance = None
        self.current_duplo_x_in_frame = None
        self.current_duplo_y_in_frame = None

        self.mission1_available_corners = [self.list_of_corners_Z1[2], self.list_of_corners_Z1[3], self.list_of_corners_Z1[4]]
        self.mission2_available_corners = self.list_of_corners_Z2.copy()
        self.number_of_duplos_collected = 0.0

        self.align_manual = False
        self.length = None

        self.prepare_exit = True
        self.exit_protection = False
        self.rotate_exit = False
        self.approach_exit = False
        self.exit_door = False

        self.prepare_deload = True
        self.deload_move = False
    
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

        elif self.state == RobotState.S1_TRANSIT_STATE:

            next_state = self.execute_S1_TRANSIT_STATE()

        elif self.state == RobotState.S1_MOVE_TO_P11:

            next_state = self.execute_S1_MOVE_TO_P11()

        elif self.state == RobotState.SALL_SEARCH_FOR_DUPLO:

            next_state = self.execute_SALL_SEARCH_FOR_DUPLO()

        elif self.state == RobotState.SALL_MOVE_TO_DUPLO:

            next_state = self.execute_SALL_MOVE_TO_DUPLO()

        elif self.state == RobotState.SALL_MOVE_TO_CLOSEST_CORNER:

            next_state = self.execute_SALL_MOVE_TO_CLOSEST_CORNER()

        elif self.state == RobotState.S1_EXIT_ROOM:

            next_state = self.execute_S1_EXIT_ROOM()

        elif self.state == RobotState.SALL_RETURN_TO_S:

            next_state = self.execute_SALL_RETURN_TO_S()

        elif self.state == RobotState.SALL_DELOAD:

            next_state = self.execute_SALL_DELOAD()

        elif self.state == RobotState.S1_TRANSIT_STATE:

            next_state = self.execute_S1_TRANSIT_STATE()

        elif self.state == RobotState.S1_SWEEP:

            next_state = self.execute_S1_SWEEP()
            
        else:

            next_state = RobotState.S_STOP

        #----- Update elapsed time -----

        if self.started_moving:

            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time

            self.elapsed_time = elapsed_time.nanoseconds * 1e-9

        #----- Check timeout -----

        if not self.running_timeout1 and self.current_mission == 1 and self.elapsed_time > self.list_of_timeouts[0]:
            
            self.running_timeout1 = True

            self.get_logger().info("TIMEOUT 1 REACHED") 

            if next_state in [RobotState.S1_MOVE_TO_P10, RobotState.S1_PRESS_BUTTON, RobotState.S1_MOVE_TO_P11]:

                next_state = RobotState.S1_TRANSIT_STATE
                self._mission1_reset_params()

            elif next_state in [RobotState.SALL_SEARCH_FOR_DUPLO, RobotState.SALL_MOVE_TO_DUPLO, RobotState.SALL_MOVE_TO_CLOSEST_CORNER]:

                next_state = RobotState.S1_EXIT_ROOM       
                self._mission_reset_params()

        #----- 2nd timeout check -----

        if not self.running_timeout2 and self.elapsed_time > self.list_of_timeouts[1]:

            self.running_timeout2 = True

            self.get_logger().info("TIMEOUT 2 REACHED")     

            next_state = RobotState.SALL_RETURN_TO_S   
            self._mission_reset_params()

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

        back_index = int((3.14 - msg.angle_min) / msg.angle_increment)
        self.back_distance = msg.ranges[back_index]

    def start_callback(self, msg):

        self.start = True
        self.stop  = False

    def stop_callback(self, msg):

        self.stop  = True
        self.start = False

        self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

    def yolov11_in_frame_callback(self, msg: Point):

            dx, dy = msg.x, msg.y

            self.current_duplo_x_in_frame, self.current_duplo_y_in_frame = dx, dy

    def yolov11_callback(self, msg: Point):

        if self.state in [RobotState.SALL_SEARCH_FOR_DUPLO, RobotState.S1_MOVE_TO_P11, RobotState.SALL_MOVE_TO_CLOSEST_CORNER, RobotState.SALL_MOVE_TO_DUPLO]:

            dx, dy = msg.x, msg.y
            area = msg.z

            if dx is not None:

                if not self.is_in_vicinity_carpet(dx, dy):

                    if area > self._best_area:

                        self._best_area = area
                        self.nearest_duplo_x = dx
                        self.nearest_duplo_y = dy

        else:

            self._best_area = -1.0
            self.nearest_duplo_x = None
            self.nearest_duplo_y = None

    def button_pose_callback(self, msg):

        if msg.data > -2.0:

            self.button_found = True
            self.button_normalized_coord = msg.data

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

    def _publish_button_image_cmd(self, command=0.0):

        msg = Float32()
        msg.data = command
        self.button_image_run_pub.publish(msg)

        