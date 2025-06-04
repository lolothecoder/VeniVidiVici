import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PointStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Empty, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import numpy as np
import math
import time

from veni_vidi_vici_bot_one.VVV_util import *
from veni_vidi_vici_bot_one.VVV_sm_node import RobotState, StateMachineNode

#----- General methods -----

def _create_pose_stamped(self, goal_x, goal_y, theta):
     
    pose_stamped_ = PoseStamped()
    pose_stamped_.header.frame_id = 'map'
    pose_stamped_.header.stamp = self.get_clock().now().to_msg()
    pose_stamped_.pose.position.x = goal_x
    pose_stamped_.pose.position.y = goal_y
    pose_stamped_.pose.orientation.z = math.sin(theta / 2.0)
    pose_stamped_.pose.orientation.w = math.cos(theta / 2.0)    

    return pose_stamped_

def _navigate_to_pose(self, goal_x=0.0, goal_y=0.0, goal_theta=0.0):

    goal_pose = self._create_pose_stamped(goal_x, goal_y, goal_theta)
    self.navigator.goToPose(goal_pose)

def _clear_costmaps(self):

    self.navigator.clearGlobalCostmap()
    self.navigator.clearLocalCostmap()

def _detect_next_duplo(self):

    #----- This function should detect the next duplo -----

    self.next_duplo['x'] = None
    self.next_duplo['y'] = None
    self.next_duplo['theta'] = None

    return False

def _get_closest_non_visited_corner(self, current_x, current_y, list_of_points):

    min_dist  = 100.0    
    min_point = None
    min_point_idx = None

    for idx, point in enumerate(list_of_points):

        distance = np.sqrt((point['x'] - current_x)**2 + (point['y'] - current_y)**2)

        if distance < min_dist:

            min_dist = distance
            min_point = point
            min_point_idx = idx

    return min_point['x'], min_point['y'], min_point['theta'], min_point_idx

#----- Execute states of the SM methods -----

def execute_S0_BOOT(self):

    self.get_logger().info("State: S0_BOOT")
    next_state = RobotState.S0_BOOT

    #----- Check for transition -----

    if self.initialized:

            next_state = RobotState.SALL_READY       

    return next_state 

def execute_SALL_READY(self):

    self.get_logger().info("State: SALL_READY")
    next_state = RobotState.SALL_READY

    #self._publish_door_servo_cmd(servo_command=[1.0])
    #self._publish_ramp_servo_cmd(servo_command=[1.0])
    #self._publish_collector_servo_cmd(collector_command=[-72.0])

    #----- Check for transition -----

    if self.current_mission == 1 and self.start and not self.stop:
         
         next_state = RobotState.S1_MOVE_TO_P10

    elif self.current_mission == 2 and self.start and not self.stop:
         
        next_state = RobotState.S2_MOVE_TO_P20

    elif self.current_mission == 3 and self.start and not self.stop:
         
        next_state = RobotState.S3

    return next_state

def execute_S1_MOVE_TO_P10(self):

    self.get_logger().info("State: S1_MOVE_TO_P10")
    next_state = RobotState.S1_MOVE_TO_P10

    #----- Check for transition -----

    p10 = self.list_of_corners_Z1[0]
    p10_x = p10['x']
    p10_y = p10['y']
    p10_theta = p10['theta']

    if not self.started_moving:

        self.start_time = self.get_clock().now()
        self.started_moving = True

    if self.wait_for_goal == False:
        
        self._navigate_to_pose(goal_x=p10_x, goal_y=p10_y, goal_theta=p10_theta)
        self.wait_for_goal = True

    else:
         
        if self.navigator.isTaskComplete():
              
            result = self.navigator.getResult()

            current_x   = self.trans.transform.translation.x
            current_y   = self.trans.transform.translation.y
            current_yaw = self.yaw

            self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

            if result == TaskResult.SUCCEEDED:
                   
                self.get_logger().info('REACHED P10')
                self.wait_for_goal = False
                next_state = RobotState.S1_PRESS_BUTTON

            else:

                self.get_logger().info('FAILED TO REACH P10')

    return next_state

def execute_S1_PRESS_BUTTON(self):

    self.get_logger().info("State: S1_PRESS_BUTTON")
    next_state = RobotState.S1_PRESS_BUTTON

    des_x = self.button_['x']
    des_y = self.button_['y']
    des_theta = self.button_['theta']
    door_thr = self.button_['door_range']

    if self.adjust_orientation:

        current_yaw = self.yaw
        rotation_goal_reached, angular_z_mn = execute_rotation(angle=des_theta, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.1, min_angular_speed=0.01, control=True)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info("ORIENTATION ADJUSTED") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.adjust_orientation = False
            self.approach_button = True

    elif self.approach_button:

        current_x   = self.trans.transform.translation.x
        current_y   = self.trans.transform.translation.y
        trans_goal_reached, linear_x_mn = execute_translation(des_x, des_y, current_x, current_y, distance_tolerance=0.1, max_linear_speed=0.1, min_linear_speed=0.01)

        if not trans_goal_reached:

            self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)

        else:

            self.get_logger().info("BUTTON APROACHED") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.approach_button = False
            self.verify_door_is_open = True

    elif self.verify_door_is_open:

        self.S1_door_open = self.left_distance > door_thr
        self.get_logger().info(f"Door range: measured={self.left_distance:.2f}, thr={door_thr:.2f}")

        if self.S1_door_open:

            self.get_logger().info("DOOR IS OPEN")
            self.verify_door_is_open = False
            self.go_back_button = True

        elif self.wait_scan_counter < 10:

            self.wait_scan_counter += 1

        elif self.num_press_attempt < 2:

            self.num_press_attempt += 1
            self.adjust_orientation = True
            self.wait_scan_counter = 0.0

        else:

            self.get_logger().info("FAILED TO OPEN THE DOOR... MISSION ABORT")
            next_state = RobotState.SALL_RETURN_TO_S

    elif self.go_back_button:

        des_x_ret = des_x - 0.4
        des_y_ret = des_y

        current_x   = self.trans.transform.translation.x
        current_y   = self.trans.transform.translation.y

        return_goal_reached, linear_x_mn = execute_translation(des_x_ret, des_y_ret, current_x, current_y, distance_tolerance=0.1, max_linear_speed=0.1, min_linear_speed=0.01)

        if not return_goal_reached:

            self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0) 

        else:
            self.get_logger().info("RETURN REACHED") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.go_back_button = False

    else:   

        next_state = RobotState.S1_MOVE_TO_P11

    return next_state 

def execute_S1_MOVE_TO_P11(self):

    self.get_logger().info("State: S1_MOVE_TO_P11")
    next_state = RobotState.S1_MOVE_TO_P11

    return next_state

#----- Attach the methods to the class -----

StateMachineNode._create_pose_stamped = _create_pose_stamped
StateMachineNode._navigate_to_pose    = _navigate_to_pose
StateMachineNode._clear_costmaps      = _clear_costmaps
StateMachineNode._detect_next_duplo   = _detect_next_duplo
StateMachineNode._get_closest_non_visited_corner  = _get_closest_non_visited_corner

StateMachineNode.execute_S0_BOOT                     = execute_S0_BOOT
StateMachineNode.execute_SALL_READY                  = execute_SALL_READY
StateMachineNode.execute_S1_MOVE_TO_P10              = execute_S1_MOVE_TO_P10
StateMachineNode.execute_S1_PRESS_BUTTON             = execute_S1_PRESS_BUTTON
StateMachineNode.execute_S1_MOVE_TO_P11              = execute_S1_MOVE_TO_P11