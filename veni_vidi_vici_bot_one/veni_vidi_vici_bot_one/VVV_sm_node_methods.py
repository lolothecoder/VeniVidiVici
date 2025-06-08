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

    if self.nearest_duplo_x is not None:

        self.next_duplo['x'] = self.nearest_duplo_x
        self.next_duplo['y'] = self.nearest_duplo_y

        temp = {}
        temp['x'] = self.nearest_duplo_x
        temp['y'] = self.nearest_duplo_y

        self.list_of_visited_duplos.append(temp)

        return True

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

def is_circle_free(self, x, y, r, threshold):
        
        if self.costmap is None:
            return False

        map_x = int((x - self.costmap.metadata.origin.position.x) / self.costmap.metadata.resolution)
        map_y = int((y - self.costmap.metadata.origin.position.y) / self.costmap.metadata.resolution)

        width = self.costmap.metadata.size_x
        height = self.costmap.metadata.size_y
        data = np.array(self.costmap.data).reshape((height, width))

        for i in range(-r, r + 1):

            for j in range(-r, r + 1):

                if i**2 + j**2 <= r**2:

                    check_x = map_x + i
                    check_y = map_y + j

                    if check_x < 0 or check_x >= width or check_y < 0 or check_y >= height:
                        return False
                    if data[check_y, check_x] > threshold:
                        return False
                    
        return True

#----- Execute states of the SM methods -----

def execute_S0_BOOT(self):

    self.get_logger().info("State: S0_BOOT")
    next_state = RobotState.S0_BOOT

    #----- Check for transition -----

    if self.initialized:

            self.get_logger().info("Transition to: SALL_READY")
            next_state = RobotState.SALL_READY    
            self.previous_state = RobotState.S0_BOOT   

    return next_state 

def execute_S_STOP(self):

    self.get_logger().info("State: S_STOP")
    next_state = RobotState.S_STOP

    return next_state  

def execute_SALL_READY(self):

    next_state = RobotState.SALL_READY

    #----- Check for transition -----

    if self.start and not self.stop:

        if not self.started_moving:

            self.start_time = self.get_clock().now()
            self.started_moving = True        

            self.current_mission = 1
            self.get_logger().info("Transition to: S1_MOVE_TO_P10")
            next_state = RobotState.S1_MOVE_TO_P10 
            self.previous_state = RobotState.SALL_READY

        else:

            self._clear_costmaps()

            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time

            elapsed_time_sec = elapsed_time.nanoseconds * 1e-9

            #----- If in ready state, and you have time, execute mission 1 again -----

            if self.current_mission == 1 and self.N_mission1_executed < 2 and not self.mission1_abandoned and self.list_of_timeouts[0] - elapsed_time_sec >= 60.0:
                
                self.current_mission = 1
                self.get_logger().info("Transition to: S1_MOVE_TO_P10")
                next_state = RobotState.S1_MOVE_TO_P10 
                self.previous_state = RobotState.SALL_READY

                #----- Reinitialize the corners of the first mission -----

                self.mission1_available_corners = [self.list_of_corners_Z1[2], self.list_of_corners_Z1[3], self.list_of_corners_Z1[4]]

            elif self.current_mission == 1 and self.mission1_abandoned:

                self.current_mission = 2

                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY
            
            #----- If in ready state, and remaining time till mission1 timeout < 1 min, start mission 2 -----

            elif self.current_mission == 1 and self.list_of_timeouts[0] - elapsed_time_sec < 60.0:          

                self.current_mission = 2

                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY

            #----- If in ready state, and remaining time till mission2 timeout > 1 min, repeat mission 2 at most 3 times -----

            elif self.current_mission == 2 and self.N_mission2_executed < 3 and self.list_of_timeouts[1] - elapsed_time_sec >= 60.0:     

                self.current_mission = 2
                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY

                self.mission2_available_corners = self.list_of_corners_Z2.copy()

    return next_state

def execute_S1_MOVE_TO_P10(self):

    next_state = RobotState.S1_MOVE_TO_P10

    #----- Check for transition -----

    p10 = self.list_of_corners_Z1[0]
    p10_x = p10['x']
    p10_y = p10['y']
    p10_theta = p10['theta']

    #----- Turn on collector to cleanout duplos in the way -----

    #self._publish_collector_servo_cmd(collector_command=[100.0])

    #-----

    if self.wait_for_goal == False:
        
        self._navigate_to_pose(goal_x=p10_x, goal_y=p10_y, goal_theta=p10_theta)
        self.wait_for_goal = True

    else:

        current_x   = self.trans.transform.translation.x
        current_y   = self.trans.transform.translation.y
        current_yaw = self.yaw

        if not self.manual_recovery:

            if self.navigator.isTaskComplete():
                
                result = self.navigator.getResult()

                self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

                if result == TaskResult.SUCCEEDED:
                    
                    self.get_logger().info('REACHED P10')
                    self.wait_for_goal = False

                    if self.N_mission1_executed < 1:

                        self.get_logger().info("Transition to: S1_PRESS_BUTTON")
                        next_state = RobotState.S1_PRESS_BUTTON

                    else:

                        self.get_logger().info("Transition to: S1_MOVE_TO_P11")
                        next_state = RobotState.S1_MOVE_TO_P11

                    self.previous_state = RobotState.S1_MOVE_TO_P10

                else:

                    self._clear_costmaps()
                    
                    self.start_yaw = current_yaw

                    self.manual_recovery = True

                    self.angle_rotate_it   = 0
                    self.angle_increment = np.pi/3.0
        
        elif self.manual_recovery:

            if self.angle_rotate_it < 6:

                rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.start_yaw + self.angle_increment, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=True)

                if not rotation_goal_reached:

                    self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

                else:
                
                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    self.start_yaw = current_yaw
                    self.angle_rotate_it += 1

            else:

                self.manual_recovery = False

                self._navigate_to_pose(goal_x=p10_x, goal_y=p10_y, goal_theta=p10_theta)
                self.wait_for_goal = True

    return next_state

def execute_S1_PRESS_BUTTON(self):

    next_state = RobotState.S1_PRESS_BUTTON

    #----- Turn off collector until room entered -----

    #self._publish_collector_servo_cmd(collector_command=[0.0])

    #-----

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.prepare_press_button:

        self.theta_prepare = math.atan2(
            (self.button_['y'] - current_y),
            (self.button_['x'] - current_x)
        )

        self.align_button = True
        self.prepare_press_button = False
    
    elif self.align_button:

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.theta_prepare, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=True)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info("ADJUSTED ORIENTATION")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.align_button = False
            self.approach_button = True

            self.remaining_distance = self.front_distance
            self.start_x = current_x
            self.start_y = current_y

    elif self.approach_button:

        trans_goal_reached, linear_x_mn = execute_translation_distance(self.remaining_distance, self.start_x, self.start_y, current_x, current_y, distance_tolerance=0.1, max_linear_speed=0.3, min_linear_speed=0.125)

        if not trans_goal_reached:

            if self.previous_linear_x_mn * linear_x_mn >= 0:

                self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                self.previous_linear_x_mn = linear_x_mn

            else:

                self.get_logger().info('OVERSHOOT DETECTED') 
                self.get_logger().info("BUTTON APROACHED")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.approach_button = False
                self.wait_for_door_to_open = True
                self.previous_linear_x_mn = 0.0      

                self.finished_action_time = self.get_clock().now()          

        else:
            
            self.get_logger().info("BUTTON APROACHED")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.approach_button = False
            self.wait_for_door_to_open = True
            self.previous_linear_x_mn = 0.0

            self.finished_action_time = self.get_clock().now()

    elif self.wait_for_door_to_open:

        elapsed_time = self.get_clock().now() - self.finished_action_time
        elsapsed_time_sec = elapsed_time.nanoseconds * 1e-9

        if elsapsed_time_sec > 3.0:
            
            self.get_logger().info('3 sec HAVE PASSED')
            self.wait_for_door_to_open = False
            self.verify_door_is_open = True

    elif self.verify_door_is_open:   

        door_thr = self.button_['door_range']

        self.S1_door_open = self.left_distance > door_thr
        self.get_logger().info(f"Door range: measured={self.left_distance:.2f}, thr={door_thr:.2f}")

        if self.S1_door_open:

            self.get_logger().info("DOOR IS OPEN")

        else:

            self.get_logger().info("DOOR NOT OPEN - MOVE BACK")
          
        self.verify_door_is_open = False
        self.go_back_button = True 

        self.start_x = current_x
        self.start_y = current_y
        self.remaining_distance = -0.75

    elif self.go_back_button:

        trans_goal_reached, linear_x_mn = execute_translation_distance(self.remaining_distance, self.start_x, self.start_y, current_x, current_y, distance_tolerance=0.1, max_linear_speed=0.3, min_linear_speed=0.125)

        if not trans_goal_reached:

            if self.previous_linear_x_mn * linear_x_mn >= 0:

                self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                self.previous_linear_x_mn = linear_x_mn

            else:

                self.get_logger().info('OVERSHOOT DETECTED') 
                self.get_logger().info("MOVED BACK")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.go_back_button = False
                self.previous_linear_x_mn = 0.0              

                if not self.S1_door_open and self.button_attempts < 3:

                    self.theta_prepare = math.atan2(
                        (self.button_['y'] - current_y),
                        (self.button_['x'] - current_x)
                    )

                    self.align_button = True
                    self.button_attempts += 1

                elif self.S1_door_open:

                    self.get_logger().info("Transition to: S1_MOVE_TO_P11")
                    next_state = RobotState.S1_MOVE_TO_P11

                    self.previous_state = RobotState.S1_PRESS_BUTTON

                else:

                    self.get_logger().info("Failed to open the door")
                    next_state = RobotState.S_STOP

                    self.previous_state = RobotState.S1_PRESS_BUTTON                    

        else:
            
            self.get_logger().info("MOVED BACK")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.go_back_button = False
            self.previous_linear_x_mn = 0.0

            if not self.S1_door_open and self.button_attempts < 3:

                self.theta_prepare = math.atan2(
                    (self.button_['y'] - current_y),
                    (self.button_['x'] - current_x)
                )

                self.align_button = True
                self.button_attempts += 1

            elif self.S1_door_open:

                self.get_logger().info("Transition to: S1_MOVE_TO_P11")
                next_state = RobotState.S1_MOVE_TO_P11

                self.previous_state = RobotState.S1_PRESS_BUTTON

            else:

                self.get_logger().info("Failed to open the door")
                next_state = RobotState.S_STOP

                self.previous_state = RobotState.S1_PRESS_BUTTON                
            
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
