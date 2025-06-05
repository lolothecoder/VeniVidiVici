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
    self._publish_collector_servo_cmd(collector_command=[-72.0])

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
    self._publish_collector_servo_cmd(collector_command=[-72.0])

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

    self._publish_collector_servo_cmd(collector_command=[-72.0])
    p10_x = 4.5
    p10_y = 4.5
    p10_theta = 3.0*np.pi/4.0

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
                next_state = RobotState.S1_MOVE_TO_P11

            else:

                self.get_logger().info('FAILED TO REACH P10')

    return next_state 

def execute_S1_MOVE_TO_P11(self):

    self.get_logger().info("State: S1_MOVE_TO_P11")
    next_state = RobotState.S1_MOVE_TO_P11
    self._publish_collector_servo_cmd(collector_command=[-0.0])
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