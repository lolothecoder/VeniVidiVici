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
import random

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

    limit_x_min  = 4.0
    limit_x_max = 12.0
    limit_y_max = 5.0
    limit_y_min = -3.0

    if self.nearest_duplo_x is not None:

        if self.nearest_duplo_x > limit_x_min and self.nearest_duplo_x < limit_x_max and self.nearest_duplo_y > limit_y_min and self.nearest_duplo_y < limit_y_max:

            self.get_logger().info("FOUND A FEASIBLE DUPLO")
            self.next_duplo['x'] = self.nearest_duplo_x
            self.next_duplo['y'] = self.nearest_duplo_y

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

def is_in_carpet(self, dx, dy):

    return dx >= 4.0 and dx <= 6.0 and dy >= -3 and dy <= 0

def is_in_vicinity_carpet(self, dx, dy):

    return dx >= 4.0 and dx <= 6.4 and dy >= -3 and dy <= 0.4

def close_to_wall(self):

    if self.front_distance <= 0.6: #0.3:        
        self.front_close = True
    else:
        self.front_close = False

    if self.back_distance <= 0.6:        
        self.back_close = True
    else:
        self.back_close = False 

    if self.left_distance <= 0.6: #0.3:      
        self.left_close = True
    else:
        self.left_close = False 

    if self.right_distance <= 0.6: #0.3:     
        self.right_close = True
    else:
        self.right_close = False 

    return self.front_close or self.back_close or self.left_close or self.right_close

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

            #self.current_mission = 2
            #next_state = RobotState.SALL_SEARCH_FOR_DUPLO
            #self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")

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

                self.mission1_available_corners = self.list_of_corners_Z12.copy()

            elif self.current_mission == 1 and self.mission1_abandoned:

                self.current_mission = 2

                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY
                self.mission2_available_corners = self.list_of_corners_Z2.copy()
            
            #----- If in ready state, and remaining time till mission1 timeout < 1 min, start mission 2 -----

            elif self.current_mission == 1 and self.list_of_timeouts[0] - elapsed_time_sec < 60.0:          

                self.current_mission = 2

                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY
                self.mission2_available_corners = self.list_of_corners_Z2.copy()

            #----- If in ready state, and remaining time till mission2 timeout > 1 min, repeat mission 2 at most 3 times -----

            elif self.current_mission == 2 and self.N_mission2_executed < 3 and self.list_of_timeouts[1] - elapsed_time_sec >= 60.0:     

                self.current_mission = 2
                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                self.previous_state = RobotState.SALL_READY

                self.mission2_available_corners = self.list_of_corners_Z22.copy()

            elif self.current_mission == 2:

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

    self._publish_collector_servo_cmd(collector_command=[-80.0])

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

                        self._clear_costmaps()

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

                rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.start_yaw + self.angle_increment, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.5, min_angular_speed=0.12, control=False)

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

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.prepare_press_button:

        self.theta_prepare = math.atan2(
            (self.button_['y'] - current_y),
            (self.button_['x'] - current_x)
        )

        self.align_wall = True
        self.prepare_press_button = False

    elif self.align_wall:

        rotation_goal_reached, angular_z_mn = execute_rotation(self.theta_prepare, current_yaw, angle_tolerance=0.05, max_angular_speed=0.2, min_angular_speed=0.12, control=True)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info("ADJUSTED WALL ORIENTATION")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.align_wall = False
            self.align_button = True
    
    elif self.align_button:

        rotation_goal_reached, angular_z_mn = execute_rotation_camera(current_x = self.button_normalized_coord, angle_tolerance=0.1, max_angular_speed=0.2, min_angular_speed=0.12, control=True)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info("ADJUSTED BUTTON ORIENTATION")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.align_button = False
            self.approach_button = True

            self.previous_lidar = self.front_distance

    elif self.approach_button:

        trans_goal_reached, linear_x_mn = execute_translation_lidar(self.front_distance, distance_tolerance=0.05, max_linear_speed=0.3, min_linear_speed=0.125)

        if not trans_goal_reached:

            if self.previous_linear_x_mn * linear_x_mn >= 0 and self.lidar_dist_counter < 20:

                self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                self.previous_linear_x_mn = linear_x_mn

                if np.abs(self.previous_lidar - self.front_distance) < 0.05:

                    self.lidar_dist_counter += 1

                else:

                    self.lidar_dist_counter = 0.0

                self.previous_lidar = self.front_distance

            elif self.previous_linear_x_mn * linear_x_mn < 0 and self.lidar_dist_counter < 20:

                self.get_logger().info('OVERSHOOT DETECTED') 
                self.get_logger().info("BUTTON APROACHED")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.approach_button = False
                self.wait_for_door_to_open = True
                self.previous_linear_x_mn = 0.0      

                self.finished_action_time = self.get_clock().now()   

            elif self.lidar_dist_counter >= 20:
                
                self.get_logger().info("ROBOT IN THE WALL")
                self.get_logger().info("BUTTON APROACHED")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.approach_button = False
                self.verify_door_is_open = True
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

        self._publish_collector_servo_cmd(collector_command=[0.0])

        elapsed_time = self.get_clock().now() - self.finished_action_time
        elsapsed_time_sec = elapsed_time.nanoseconds * 1e-9
        self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

        if elsapsed_time_sec > 6.0:
            
            self.get_logger().info('6 sec HAVE PASSED')
            self.wait_for_door_to_open = False
            self.verify_door_is_open = True
            self.finished_action_time = None

    elif self.verify_door_is_open:   

        door_thr = self.button_['door_range']

        left_distance = self.left_distance
        if math.isinf(self.left_distance): left_distance = 100.0

        self.S1_door_open = left_distance > door_thr
        self.get_logger().info(f"Door range: measured={self.left_distance:.2f}, thr={door_thr:.2f}")
        
        if self.S1_door_open:

            self.get_logger().info("DOOR IS OPEN - MOVE BACK")

        else:

            self.get_logger().info("DOOR NOT OPEN - MOVE BACK")
          
        self.verify_door_is_open = False
        self.go_back_button = True 

        self.start_x = current_x
        self.start_y = current_y
        self.remaining_distance = -0.9

    elif self.go_back_button:

        self._publish_collector_servo_cmd(collector_command=[80.0])

        if not self.manual_recovery:

            trans_goal_reached, linear_x_mn = execute_translation_distance(self.remaining_distance, self.start_x, self.start_y, current_x, current_y, distance_tolerance=0.1, max_linear_speed=0.3, min_linear_speed=0.125)

            if not trans_goal_reached:

                if self.previous_linear_x_mn * linear_x_mn >= 0:

                    self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                    self.previous_linear_x_mn = linear_x_mn

                else:

                    self.get_logger().info('OVERSHOOT DETECTED') 
                    self.get_logger().info("MOVED BACK")
                    self._publish_collector_servo_cmd(collector_command=[0.0])
                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    
                    self.previous_linear_x_mn = 0.0              

                    if not self.S1_door_open and self.button_attempts < 3:

                        self.button_attempts += 1
                        self._clear_costmaps()
                        
                        self.manual_recovery = True
                        self._clear_costmaps()                  
                        self.start_yaw = current_yaw
                        self.angle_rotate_it   = 0
                        self.angle_increment = np.pi/3.0

                    elif self.S1_door_open:

                        self.get_logger().info("Transition to: S1_MOVE_TO_P11")
                        next_state = RobotState.S1_MOVE_TO_P11

                        self.navigator._clear_costmaps()  
                        self.go_back_button = False
                        self.prepare_press_button = True
                        self.previous_state = RobotState.S1_PRESS_BUTTON

                    else:

                        self.mission1_abandoned = True

                        self.get_logger().info("Failed to open the door")
                        self.get_logger().info("Transition to: S1_TRANSIT_STATE")
                        next_state = RobotState.S1_TRANSIT_STATE

                        self.go_back_button = False
                        self.prepare_press_button = True
                        self.previous_state = RobotState.S1_PRESS_BUTTON                    

            else:
                
                self.get_logger().info("MOVED BACK")
                self._publish_collector_servo_cmd(collector_command=[0.0])
                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

                self.previous_linear_x_mn = 0.0

                if not self.S1_door_open and self.button_attempts < 3:

                    self.button_attempts += 1

                    self.manual_recovery = True
                    self._clear_costmaps()                  
                    self.start_yaw = current_yaw
                    self.angle_rotate_it   = 0
                    self.angle_increment = np.pi/3.0

                elif self.S1_door_open:

                    self.navigator.clearLocalCostmap()
                    self.get_logger().info("Transition to: S1_MOVE_TO_P11")
                    next_state = RobotState.S1_MOVE_TO_P11

                    self.go_back_button = False
                    self.prepare_press_button = True
                    self.previous_state = RobotState.S1_PRESS_BUTTON

                else:

                    self.mission1_abandoned = True

                    self.get_logger().info("Failed to open the door")
                    self.get_logger().info("Transition to: S1_TRANSIT_STATE")
                    next_state = RobotState.S1_TRANSIT_STATE

                    self.go_back_button = False
                    self.prepare_press_button = True
                    self.previous_state = RobotState.S1_PRESS_BUTTON                

        elif self.manual_recovery:

            if self.angle_rotate_it < 6:

                rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.start_yaw + self.angle_increment, current_theta=current_yaw, angle_tolerance=0.2, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

                if not rotation_goal_reached:

                    self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

                else:
                
                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    self.start_yaw = current_yaw
                    self.angle_rotate_it += 1

            else:

                self.manual_recovery = False
                self.go_back_button = False
                self.prepare_press_button = True
                self.navigator.clearLocalCostmap()

                dist_to_button = math.sqrt((self.button_['y'] - current_y)**2 + (self.button_['x'] - current_x)**2)

                self.get_logger().info(f"Distance to button: {dist_to_button}")

                if dist_to_button >= 0.6:
                    
                    next_state = RobotState.S1_MOVE_TO_P10
                    self.get_logger().info("Transition to: S1_MOVE_TO_P10")
                    self.previous_state = RobotState.S1_PRESS_BUTTON 

    return next_state 

def execute_S1_TRANSIT_STATE(self):

    next_state = RobotState.S1_TRANSIT_STATE
    
    t_x = self.T_pose['x']
    t_y = self.T_pose['y']
    t_theta = self.T_pose['theta']

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.wait_for_goal == False:
        
        self._navigate_to_pose(goal_x=t_x, goal_y=t_y, goal_theta=t_theta)
        self.wait_for_goal = True

    else:

        if self.navigator.isTaskComplete():
            
            result = self.navigator.getResult()

            self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

            if result == TaskResult.SUCCEEDED:
                
                self.get_logger().info('REACHED TRANSIENT POSE')

                self.wait_for_goal = False

                self.current_mission = 2
                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO

                self.previous_state = RobotState.S1_TRANSIT_STATE

            else:

                self.navigator.clearLocalCostmap()
                self.current_mission = 2
                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                                    
    return next_state

def execute_S1_MOVE_TO_P11(self):

    next_state = RobotState.S1_MOVE_TO_P11

    self._publish_collector_servo_cmd(collector_command=[-80.0])

    p11 = self.list_of_corners_Z1[1]
    p11_x = p11['x']
    p11_y = p11['y']
    p11_theta = p11['theta']

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.wait_for_goal == False:
        
        self._navigate_to_pose(goal_x=p11_x, goal_y=p11_y, goal_theta=p11_theta)
        self.wait_for_goal = True   

        self.door_counter = 0.0

    else:

        if self.navigator.isTaskComplete():
            
            result = self.navigator.getResult()

            self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

            if result == TaskResult.SUCCEEDED:
                
                self.get_logger().info('REACHED P11')
                self.wait_for_goal = False

                #next_state = RobotState.S1_SWEEP
                #self.get_logger().info("Transition to: S1_SWEEP")

                self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                next_state = RobotState.SALL_SEARCH_FOR_DUPLO

                self.previous_state = RobotState.S1_MOVE_TO_P11

            else:

                self.navigator.clearLocalCostmap()

                self._navigate_to_pose(goal_x=p11_x, goal_y=p11_y, goal_theta=p11_theta)
                self.wait_for_goal = True 

                self.door_counter += 1

                if self.door_counter == 10:
                    
                    next_state = RobotState.S1_MOVE_TO_P10
                    self.get_logger().info("Door is actually closed... try again")


    return next_state

def execute_S1_SWEEP(self):

    next_state = RobotState.S1_SWEEP

    #----- Turn on collector to cleanout duplos in the way -----

    self._publish_collector_servo_cmd(collector_command=[-80.0])

    #-----

    if self.first_time_sweep:

        self.first_time_sweep = False
        self.list_of_available_setpoints = self.list_of_mission1_hardcoded_points.copy()

    if self.wait_for_goal == False:

        self.get_logger().info(f"Remaining setpoints: {len(self.list_of_available_setpoints)}")

        if len(self.list_of_available_setpoints) > 0:

            next_point = self.list_of_available_setpoints[0]
            self._navigate_to_pose(goal_x=next_point[0], goal_y=next_point[1], goal_theta=next_point[2])
            self.wait_for_goal = True

        else:

            next_state = RobotState.S1_EXIT_ROOM
            self.get_logger().info("Transition to state: RobotState.S1_EXIT_ROOM")

    else:

        if self.navigator.isTaskComplete():

            self.navigator.clearLocalCostmap()
                
            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                    
                self.get_logger().info('REACHED SETPOINT')
                
            else:

                self.get_logger().info('REACHED SETPOINT')
                
            self.wait_for_goal = False 
            del self.list_of_available_setpoints[0]

    return next_state
        
def execute_SALL_SEARCH_FOR_DUPLO(self):

    next_state = RobotState.SALL_SEARCH_FOR_DUPLO

    #----- Turn off the collector while searching -----

    self._publish_collector_servo_cmd(collector_command=[80.0])

    #-----

    p_value = random.random()

    if self.current_mission == 1:

        p_thr = 0.3

    elif self.current_mission == 2:

        p_thr = 0.8

    if p_value < p_thr:

        next_state = RobotState.SALL_MOVE_TO_CLOSEST_CORNER
        self.get_logger().info("Transition to: SALL_MOVE_TO_CLOSEST_CORNER")
        self.previous_state = RobotState.SALL_SEARCH_FOR_DUPLO

        return next_state

    #-----

    if not self.duplo_found and self.previous_state in [RobotState.SALL_MOVE_TO_CLOSEST_CORNER, RobotState.S1_MOVE_TO_P11] and not self.estimating_duplo_position:

        self.estimating_duplo_position = True

    elif not self.duplo_found and not self.adjust_orientation and not self.estimating_duplo_position:

        self.adjust_orientation = True

        if self.front_distance >= 1.5:
            
            self.get_logger().info("NO SEARCH ADJUSTEMENT")

            # yaw = self.yaw
            # yaw = normalize_angle(yaw)

            # self.des_theta_left  = yaw + np.pi/8.0
            # self.des_theta_right = yaw - np.pi/8.0
            # self.des_theta_mid   = yaw

            # self.turn_mid = True
            # #self.turn_left = True

            self.estimating_duplo_position = True

        elif self.left_distance < 0.5 and self.right_distance >= 0.5:

            self.get_logger().info("SEARCH: TURN RIGHT")
            yaw = self.yaw - np.pi/2
            yaw = normalize_angle(yaw)

            self.des_theta_mid   = yaw

            self.turn_mid = True
            

        elif self.left_distance < 0.5 and self.right_distance < 0.5:

            self.get_logger().info("SEARCH: TURN AROUND")
            yaw = self.yaw - np.pi

            yaw = normalize_angle(yaw)

            self.des_theta_mid   = yaw

            self.turn_mid = True
            

        elif self.left_distance >= 0.5 and self.right_distance >= 0.5:
            
            if self.left_distance >= self.right_distance:
            
                self.get_logger().info("SEARCH: TURN LEFT")
                yaw = self.yaw + np.pi/2

            else:

                self.get_logger().info("SEARCH: TURN RIGHT")
                yaw = self.yaw - np.pi/2

            yaw = normalize_angle(yaw)

            self.des_theta_mid   = yaw

            self.turn_mid = True
            


        elif self.left_distance >= 0.5 and self.right_distance < 0.5:

            self.get_logger().info("SEARCH: TURN LEFT")
            yaw = self.yaw + np.pi/2
            yaw = normalize_angle(yaw)

            self.des_theta_mid   = yaw

            self.turn_mid = True
            

    elif self.adjust_orientation and self.turn_left and not self.estimating_duplo_position:

        current_yaw = self.yaw

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.des_theta_left, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)
            self._publish_collector_servo_cmd(collector_command=[0.0])

        else:
            
            self.get_logger().info("LOOKED LEFT") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.turn_left  = False
            self.turn_right = True

    elif self.adjust_orientation and self.turn_right and not self.estimating_duplo_position:

        current_yaw = self.yaw

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.des_theta_right, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)
            self._publish_collector_servo_cmd(collector_command=[0.0])

        else:
            
            self.get_logger().info("LOOKED RIGHT") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.turn_right = False
            self.turn_mid = True 

    elif self.adjust_orientation and self.turn_mid and not self.estimating_duplo_position:

        current_yaw = self.yaw

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.des_theta_mid, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)
            self._publish_collector_servo_cmd(collector_command=[0.0])

        else:
            
            self.get_logger().info("FINISHED LOOKING") 
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.turn_mid = False

            self.adjust_orientation = False
            self.estimating_duplo_position = True

    elif self.estimating_duplo_position:

        self.estimating_duplo_position = False

        self.duplo_found = self._detect_next_duplo()
        
        if self.duplo_found:

            self.current_duplo_x = self.next_duplo['x']
            self.current_duplo_y = self.next_duplo['y']

            self.get_logger().info(f"DUPLO DETECTED: x={self.current_duplo_x} y={self.current_duplo_y}")
            
            next_state = RobotState.SALL_MOVE_TO_DUPLO
            self.get_logger().info("Transition to: SALL_MOVE_TO_DUPLO")
            self.previous_state = RobotState.SALL_SEARCH_FOR_DUPLO

        else:

            next_state = RobotState.SALL_MOVE_TO_CLOSEST_CORNER
            self.get_logger().info("Transition to: SALL_MOVE_TO_CLOSEST_CORNER")
            self.previous_state = RobotState.SALL_SEARCH_FOR_DUPLO

    return next_state

def execute_SALL_MOVE_TO_DUPLO(self):

    next_state = RobotState.SALL_MOVE_TO_DUPLO

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    self.distance_to_duplo = math.sqrt((self.current_duplo_y - current_y)**2 + (self.current_duplo_x - current_x)**2)

    if self.current_mission == 2:

        self._clear_costmaps()

    #----- Turn on collector to cleanout duplos in the way -----

    self._publish_collector_servo_cmd(collector_command=[-80.0])

    #-----

    if self.duplo_found:

        if self.distance_to_duplo <= 0.8:

            self.go_straight_to_duplo = True
            self.nav2_to_duplo        = False

        else:

            self.go_straight_to_duplo = False
            self.nav2_to_duplo        = True    

        self.current_duplo_theta = math.atan2(
            (self.current_duplo_y - current_y),
            (self.current_duplo_x - current_x)
        )     

        self.duplo_found = False
        self.align_with_duplo = True

        if self.current_duplo_x_in_frame is None: 
            
            self.get_logger().info("MANUAL Aligning with duplo..")
            self.align_manual = True

        else:

            self.get_logger().info("AUTOMATIC Aligning with duplo..")
            self.align_manual = False

        #----- Manual -----

        if self.current_mission == 1: self.go_straight_to_duplo = True

    elif self.align_with_duplo:

        if self.align_manual:

            rotation_goal_reached, angular_z_mn = execute_rotation(self.current_duplo_theta, current_yaw, angle_tolerance=0.15, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

        else:

            rotation_goal_reached, angular_z_mn = execute_rotation_camera(current_x = self.current_duplo_x_in_frame, angle_tolerance=0.15, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

        #-----

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
                
            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)  
            self.align_with_duplo = False  
            self.align_manual = False

            self.current_duplo_theta = current_yaw

            if self.go_straight_to_duplo:

                self.get_logger().info("Go straight to duplo...")
                self.length = self.distance_to_duplo
                self.start_distance = self.front_distance
                
            elif self.nav2_to_duplo:

                self.get_logger().info("NAV2 to duplo...")

    elif self.go_straight_to_duplo:

        if self.is_in_vicinity_carpet(current_x, current_y): self.get_logger().info("In the vicinity of carpet") 

        if not self.close_to_wall() and not self.is_in_vicinity_carpet(current_x, current_y):

            trans_goal_reached, linear_x_mn = execute_translation_laser(curr_dist=self.front_distance, start_dist=self.start_distance, length=self.length, distance_tolerance=0.05, max_linear_speed=0.3, min_linear_speed=0.15)

            if not trans_goal_reached:

                if self.previous_linear_x_mn * linear_x_mn >= 0:

                    self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                    self.previous_linear_x_mn = linear_x_mn

                else:

                    self.get_logger().info("OVERSHOOT DETECTED") 
                    self.get_logger().info("DUPLO EXECUTED")

                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0) 
                    self.number_of_duplos_collected += 1

                    if self.number_of_duplos_collected < self.duplos_capacity:

                        next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                        self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")

                    else:

                        if self.current_mission == 1:
                            next_state = RobotState.S1_EXIT_ROOM
                            self.get_logger().info("Robot full! Transition to: S1_EXIT_ROOM")
                        elif self.current_mission == 2:
                            next_state = RobotState.SALL_RETURN_TO_S
                            self.get_logger().info("Robot full! Transition to: SALL_RETURN_TO_S")

                    self.previous_state = RobotState.SALL_MOVE_TO_DUPLO 

                    self._best_area = -1.0
                    self.nearest_duplo_x = None
                    self.nearest_duplo_y = None

                    self.previous_linear_x_mn = 0.0
                    self.go_straight_to_duplo = False
                    self.finished_action_time = None
                    self.length = None
                    self.start_distance = None

            else:

                self.get_logger().info("DUPLO EXECUTED")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.number_of_duplos_collected += 1

                if self.number_of_duplos_collected < self.duplos_capacity:

                    next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                    self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")

                else:

                    if self.current_mission == 1:
                        next_state = RobotState.S1_EXIT_ROOM
                        self.get_logger().info("Robot full! Transition to: S1_EXIT_ROOM")
                    elif self.current_mission == 2:
                        next_state = RobotState.SALL_RETURN_TO_S
                        self.get_logger().info("Robot full! Transition to: SALL_RETURN_TO_S")

                self._best_area = -1.0
                self.nearest_duplo_x = None
                self.nearest_duplo_y = None

                self.previous_state = RobotState.SALL_MOVE_TO_DUPLO 

                self.previous_linear_x_mn = 0.0   
                self.go_straight_to_duplo = False 
                self.finished_action_time = None
                self.length = None
                self.start_distance = None

        else:

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.previous_linear_x_mn = 0.0
            self.go_straight_to_duplo = False
            self.finished_action_time = None
            self.length = None
            self.start_distance = None

            next_state = RobotState.SALL_SEARCH_FOR_DUPLO
            self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
            self.previous_state = RobotState.SALL_MOVE_TO_DUPLO

            self._best_area = -1.0
            self.nearest_duplo_x = None
            self.nearest_duplo_y = None
            
    elif self.nav2_to_duplo:

        if self.wait_for_goal == False:
            
            self._navigate_to_pose(goal_x=self.current_duplo_x, goal_y=self.current_duplo_y, goal_theta=self.current_duplo_theta)
            self.wait_for_goal = True   

        else:

            if self.distance_to_duplo <= 0.8 and self.current_duplo_x_in_frame is not None:

                self.get_logger().info(f"Distance: {self.distance_to_duplo}")

                self.navigator.cancelTask()
                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

                self.get_logger().info("CLOSE ENOUGH... Align with duplo")

                self.align_with_duplo = False
                self.go_straight_to_duplo = True
                self.nav2_to_duplo = False
                self.wait_for_goal = False

            else:

                if self.navigator.isTaskComplete():

                    self.nav2_to_duplo = False
                    self.wait_for_goal = False 

                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        
                        if self.current_duplo_x_in_frame is not None:

                            self.align_with_duplo = False
                            self.go_straight_to_duplo = True

                        else:

                            self.navigator.clearLocalCostmap()
                            next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                            self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                            self.previous_state = RobotState.SALL_MOVE_TO_DUPLO
                            
                            self._best_area = -1.0
                            self.nearest_duplo_x = None
                            self.nearest_duplo_y = None

                    else:
                        
                        self.navigator.clearLocalCostmap()
                        next_state = RobotState.SALL_SEARCH_FOR_DUPLO
                        self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
                        self.previous_state = RobotState.SALL_MOVE_TO_DUPLO

                        self._best_area = -1.0
                        self.nearest_duplo_x = None
                        self.nearest_duplo_y = None

    return next_state

def execute_SALL_MOVE_TO_CLOSEST_CORNER(self):

    next_state = RobotState.SALL_MOVE_TO_CLOSEST_CORNER

    #----- Turn on the collector -----

    self._publish_collector_servo_cmd(collector_command=[-80.0])

    #----

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.prepare_corner:

        if self.current_mission == 1:
            list_of_corners = self.mission1_available_corners

        elif self.current_mission == 2:
            list_of_corners = self.mission2_available_corners

        elif self.current_mission == 3:
            list_of_corners = self.mission3_available_corners

        if len(list_of_corners) > 0:

            self.corner_x, self.corner_y, self.corner_theta, self.corner_idx = self._get_closest_non_visited_corner(current_x, current_y, list_of_corners)
            self.corner_goal_theta = math.atan2(
                (self.corner_y - current_y),
                (self.corner_x - current_x)
            )

            self.distance_to_corner = math.hypot(current_x - self.corner_x, current_y - self.corner_y)

            if self.distance_to_corner <= 0.8:

                self.get_logger().info("GOING MANUALLY TO CORNER")
                self.go_straight_to_corner = True
                self.nav2_to_corner        = False

            else:

                self.get_logger().info("GOING NAV2 TO CORNER")
                self.go_straight_to_corner = False
                self.nav2_to_corner        = True      

            self.prepare_corner = False
            self.align_corner = True  

        else:

            self.get_logger().info("NO MORE CORNERS")

            if self.current_mission == 1:

                next_state = RobotState.S1_EXIT_ROOM
                self.get_logger().info("Transition to: S1_EXIT_ROOM")   

            elif self.current_mission == 2:

                next_state = RobotState.SALL_RETURN_TO_S
                self.get_logger().info("Transition to: SALL_RETURN_TO_S") 
                
            self.previous_state = RobotState.SALL_MOVE_TO_CLOSEST_CORNER 

        #----- Fully manual -----

        if self.current_mission == 1: self.go_straight_to_corner = True

    elif self.align_corner:
    
        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.corner_goal_theta, current_theta=current_yaw, angle_tolerance=0.1, max_angular_speed=0.3, min_angular_speed=0.12, control=False)            

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info('ALIGNED WITH CORNER')

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.align_corner = False 

            self.length = max(self.front_distance - 0.0, 0.0)
            self.start_distance = self.front_distance

    elif self.go_straight_to_corner:

        if not self.close_to_wall() and not self.is_in_vicinity_carpet(current_x, current_y):

            trans_goal_reached, linear_x_mn = execute_translation_laser(curr_dist=self.front_distance, start_dist=self.start_distance, length=self.length, distance_tolerance=0.05, max_linear_speed=0.2, min_linear_speed=0.15)

            if not trans_goal_reached:

                if self.previous_linear_x_mn * linear_x_mn >= 0:

                    self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                    self.previous_linear_x_mn = linear_x_mn

                else:
                    
                    self.get_logger().info('OVERSHOOT DETECTED') 
                    self.get_logger().info("REACHED CORNER")
                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    self.go_straight_to_corner = False
                    self.match_corner_orientation = True

                    self.previous_linear_x_mn = 0.0
                    self.finished_action_time = None
                    self.length = None
                    self.start_distance = None
                    
            else:

                self.get_logger().info("REACHED CORNER") 
                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.go_straight_to_corner = False
                self.match_corner_orientation = True

                self.previous_linear_x_mn = 0.0  
                self.finished_action_time = None    
                self.length = None
                self.start_distance = None

        else:

            self.go_straight_to_corner = False
            self.match_corner_orientation = True

            self.previous_linear_x_mn = 0.0
            self.finished_action_time = None
            self.length = None
            self.start_distance = None
            
    elif self.nav2_to_corner:

        if not self.wait_for_goal:

            self._navigate_to_pose(goal_x=self.corner_x, goal_y=self.corner_y, goal_theta=self.corner_goal_theta) 
            self.wait_for_goal = True       

        else:  

            if self.navigator.isTaskComplete():
            
                result = self.navigator.getResult()

                if result == TaskResult.SUCCEEDED:
                        
                    self.get_logger().info('REACHED CORNER')
                    self.wait_for_goal = False
                    self.nav2_to_corner = False
                    self.match_corner_orientation = True                  

                else:

                    self.navigator.clearLocalCostmap()
                    self.wait_for_goal = False
                    self.nav2_to_corner = False
                    self.match_corner_orientation = True 

    elif self.match_corner_orientation:

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.corner_theta, current_theta=current_yaw, angle_tolerance=0.1, max_angular_speed=0.5, min_angular_speed=0.12, control=False)            

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:

            self.navigator.clearLocalCostmap()

            self.get_logger().info('CORNER ORIENTATION MATCHED')

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.match_corner_orientation = False

            next_state = RobotState.SALL_SEARCH_FOR_DUPLO
            self.get_logger().info("Transition to: SALL_SEARCH_FOR_DUPLO")
            self.previous_state = RobotState.SALL_MOVE_TO_CLOSEST_CORNER

            if self.current_mission == 1 and len(self.mission1_available_corners) > 0 :
                del self.mission1_available_corners[self.corner_idx] 

            if self.current_mission == 2 and len(self.mission2_available_corners) > 0 :
                del self.mission2_available_corners[self.corner_idx] 

            #----- We don't remove the corners for the second mission -----
                           
            self.prepare_corner = True

    return next_state 

def execute_S1_EXIT_ROOM(self):

    next_state = RobotState.S1_EXIT_ROOM
    
    pEX = self.list_of_corners_Z1[1]

    pEX_x = pEX['x'] - 0.3
    pEX_y = pEX['y'] + 0.3
    pEX_theta = -np.pi/4

    p10 = self.list_of_corners_Z1[0]
    p10_x = p10['x']
    p10_y = p10['y']
    p10_theta = -np.pi/2

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.prepare_exit:

        if self.exit_protection:

            self.rotate_exit = True

            self._clear_costmaps()
                    
            self.start_yaw = current_yaw
            self.angle_rotate_it   = 0
            self.angle_increment = np.pi/3.0

        else:

            self.navigator.clearLocalCostmap()
            self.approach_exit = True

        self.prepare_exit = False

    elif self.rotate_exit:

        if self.angle_rotate_it < 6:

            rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.start_yaw + self.angle_increment, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

            if not rotation_goal_reached:

                self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

            else:
            
                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.start_yaw = current_yaw
                self.angle_rotate_it += 1

        else:

            self.rotate_exit = False
            self.approach_exit = True

            self._navigate_to_pose(goal_x=pEX_x, goal_y=pEX_y, goal_theta=pEX_theta)

    elif self.approach_exit:

        if not self.manual_recovery:

            if self.navigator.isTaskComplete():
                
                result = self.navigator.getResult()

                self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

                if result == TaskResult.SUCCEEDED:
                    
                    self.get_logger().info('REACHED EXIT POINT')
                    self.approach_exit = False
                    self.exit_door = True

                    self._navigate_to_pose(goal_x=p10_x, goal_y=p10_y, goal_theta=p10_theta)

                else:

                    self._clear_costmaps()
                    
                    self.start_yaw = current_yaw

                    self.manual_recovery = True

                    self.angle_rotate_it   = 0
                    self.angle_increment = np.pi/3.0
        
        elif self.manual_recovery:

            self.manual_recovery = False
            self._navigate_to_pose(goal_x=pEX_x, goal_y=pEX_y, goal_theta=pEX_theta)

    elif self.exit_door:

        #----- Turn off the collector for random collecting -----

        self._publish_collector_servo_cmd(collector_command=[0.0])

        #-----

        if not self.manual_recovery:

            if self.navigator.isTaskComplete():
                
                result = self.navigator.getResult()

                self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

                if result == TaskResult.SUCCEEDED:
                    
                    self.get_logger().info('EXITED ROOM')
                    self.exit_door = False
                    self.prepare_exit = True

                    next_state = RobotState.SALL_RETURN_TO_S
                    self.get_logger().info("Transition to: SALL_RETURN_TO_S")

                    self.previous_state = RobotState.S1_EXIT_ROOM

                else:

                    self._clear_costmaps()
                    
                    self.start_yaw = current_yaw

                    self.manual_recovery = True

                    self.angle_rotate_it   = 0
                    self.angle_increment = np.pi/3.0
        
        elif self.manual_recovery:

            self.manual_recovery = False
            self._navigate_to_pose(goal_x=p10_x, goal_y=p10_y, goal_theta=p10_theta)

    return next_state

def execute_SALL_RETURN_TO_S(self):

    next_state = RobotState.SALL_RETURN_TO_S

    s_x = self.S_pose['x']
    s_y = self.S_pose['y']
    s_theta = self.S_pose['theta']

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    #----- Turn on the collector for random collecting if not super close -----

    dist_to_S = np.sqrt((s_x - current_x)**2 + (s_y - current_y)**2)

    if dist_to_S > 1.0:

        self._publish_collector_servo_cmd(collector_command=[-80.0])    

    else:

        self._publish_collector_servo_cmd(collector_command=[80.0]) 

    #-----

    if self.wait_for_goal == False:
        
        self._navigate_to_pose(goal_x=s_x, goal_y=s_y, goal_theta=s_theta)
        self.wait_for_goal = True

    else:

        if not self.manual_recovery:

            if self.navigator.isTaskComplete():
                
                result = self.navigator.getResult()

                self.get_logger().info(f"Current pose in map: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f} rad")

                if result == TaskResult.SUCCEEDED:
                    
                    self.get_logger().info('REACHED S')

                    self.wait_for_goal = False

                    next_state = RobotState.SALL_DELOAD
                    self.get_logger().info("Transition to: SALL_DELOAD")
                    self.previous_state = RobotState.SALL_RETURN_TO_S

                else:

                    self._clear_costmaps()
                    
                    self.start_yaw = current_yaw

                    self.manual_recovery = True

                    self.angle_rotate_it   = 0
                    self.angle_increment = np.pi/3.0
        
        elif self.manual_recovery:

            if self.angle_rotate_it < 6:

                rotation_goal_reached, angular_z_mn = execute_rotation(angle=self.start_yaw + self.angle_increment, current_theta=current_yaw, angle_tolerance=0.05, max_angular_speed=0.3, min_angular_speed=0.12, control=False)

                if not rotation_goal_reached:

                    self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

                else:
                
                    self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                    self.start_yaw = current_yaw
                    self.angle_rotate_it += 1

            else:

                self.manual_recovery = False

                self._navigate_to_pose(goal_x=s_x, goal_y=s_y, goal_theta=s_theta)
                self.wait_for_goal = True

    return next_state

def execute_SALL_DELOAD(self):

    next_state = RobotState.SALL_DELOAD

    deload_theta = -np.pi/4

    deload_x = self.S_pose['x'] + 0.5
    deload_y = self.S_pose['y'] - 0.5

    current_x   = self.trans.transform.translation.x
    current_y   = self.trans.transform.translation.y
    current_yaw = self.yaw

    if self.prepare_deload:

        rotation_goal_reached, angular_z_mn = execute_rotation(angle=deload_theta, current_theta=current_yaw, angle_tolerance=0.1, max_angular_speed=0.8, min_angular_speed=0.12, control=False)

        if not rotation_goal_reached:

            self._publish_cmd_vel(linear_x=0.0, angular_z=angular_z_mn)

        else:
            
            self.get_logger().info("FINISHED ALIGNING DELOAD")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.prepare_deload  = False
            self.deload_move = True  

            #----- Open the door and up the ramp -----

            self._publish_door_servo_cmd   (servo_command=[1.0])    
            self._publish_ramp_servo_cmd   (servo_command=[1.0]) 

    elif self.deload_move:

        trans_goal_reached, linear_x_mn = execute_translation(deload_x, deload_y, current_x, current_y, current_yaw, distance_tolerance=0.05, max_linear_speed=0.2, min_linear_speed=0.125)

        if not trans_goal_reached:

            if self.previous_linear_x_mn * linear_x_mn >= 0:

                self._publish_cmd_vel(linear_x=linear_x_mn, angular_z=0.0)
                self.previous_linear_x_mn = linear_x_mn

            else:
                
                self.get_logger().info('OVERSHOOT DETECTED') 
                self.get_logger().info("FINISHED MOVING")

                self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
                self.deload_move = False
                self.prepare_deload = True

                self._publish_door_servo_cmd   (servo_command=[0.0])    
                self._publish_ramp_servo_cmd   (servo_command=[0.0]) 

                if   self.current_mission == 1: self.N_mission1_executed += 1
                elif self.current_mission == 2: self.N_mission2_executed += 1 
                elif self.current_mission == 3: self.N_mission3_executed += 1               

                next_state = RobotState.SALL_READY

                self.get_logger().info('FINISHED DELOAD')   
                self.get_logger().info("Transition to: SALL_READY")
                self.number_of_duplos_collected = 0.0

                self.previous_linear_x_mn = 0.0
                self.previous_state = RobotState.SALL_DELOAD
        else:

            self.get_logger().info("FINISHED MOVING")

            self._publish_cmd_vel(linear_x=0.0, angular_z=0.0)
            self.deload_move = False
            self.prepare_deload = True

            self._publish_door_servo_cmd   (servo_command=[0.0])    
            self._publish_ramp_servo_cmd   (servo_command=[0.0]) 

            if   self.current_mission == 1: self.N_mission1_executed += 1
            elif self.current_mission == 2: self.N_mission2_executed += 1 
            elif self.current_mission == 3: self.N_mission3_executed += 1               

            next_state = RobotState.SALL_READY

            self.get_logger().info('FINISHED DELOAD')   
            self.get_logger().info("Transition to: SALL_READY")
            self.number_of_duplos_collected = 0.0

            self.previous_linear_x_mn = 0.0
            self.previous_state = RobotState.SALL_DELOAD

    return next_state

#----- Attach the methods to the class -----

StateMachineNode._create_pose_stamped = _create_pose_stamped
StateMachineNode._navigate_to_pose    = _navigate_to_pose
StateMachineNode._clear_costmaps      = _clear_costmaps
StateMachineNode._detect_next_duplo   = _detect_next_duplo
StateMachineNode._get_closest_non_visited_corner  = _get_closest_non_visited_corner
StateMachineNode.is_in_carpet                     = is_in_carpet
StateMachineNode.close_to_wall                    = close_to_wall
StateMachineNode.is_in_vicinity_carpet            = is_in_vicinity_carpet

StateMachineNode.execute_S0_BOOT                     = execute_S0_BOOT
StateMachineNode.execute_S_STOP                      = execute_S_STOP
StateMachineNode.execute_SALL_READY                  = execute_SALL_READY
StateMachineNode.execute_S1_MOVE_TO_P10              = execute_S1_MOVE_TO_P10
StateMachineNode.execute_S1_PRESS_BUTTON             = execute_S1_PRESS_BUTTON
StateMachineNode.execute_S1_TRANSIT_STATE            = execute_S1_TRANSIT_STATE
StateMachineNode.execute_S1_MOVE_TO_P11              = execute_S1_MOVE_TO_P11
StateMachineNode.execute_SALL_SEARCH_FOR_DUPLO       = execute_SALL_SEARCH_FOR_DUPLO
StateMachineNode.execute_SALL_MOVE_TO_DUPLO          = execute_SALL_MOVE_TO_DUPLO
StateMachineNode.execute_SALL_MOVE_TO_CLOSEST_CORNER = execute_SALL_MOVE_TO_CLOSEST_CORNER
StateMachineNode.execute_S1_EXIT_ROOM                = execute_S1_EXIT_ROOM
StateMachineNode.execute_SALL_RETURN_TO_S            = execute_SALL_RETURN_TO_S
StateMachineNode.execute_SALL_DELOAD                 = execute_SALL_DELOAD
StateMachineNode.execute_S1_SWEEP                    = execute_S1_SWEEP
