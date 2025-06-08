import math
import numpy as np

#----- Define auxiliary funcitions -----

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def normalize_angle(angle):

    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi

    return angle

def angle_difference(target, current):

    diff = (target - current) % (2 * np.pi)
    if diff > np.pi:
        diff -= 2 * np.pi

    return diff

def execute_rotation(angle, current_theta, angle_tolerance=0.07, max_angular_speed=0.5, min_angular_speed=0.1, control=True):

    current_theta = normalize_angle(current_theta)

    target_theta = normalize_angle(angle)
    angle_diff   = angle_difference(target_theta, current_theta)

    if control:

        angular_speed = np.exp(np.abs(angle_diff)*2-1) * np.abs(max_angular_speed)
        angular_speed = max(min(angular_speed, max_angular_speed), min_angular_speed)

    else:

        angular_speed = max_angular_speed

    target_theta_speed = 0 if abs(angle_diff) < angle_tolerance else np.sign(angle_diff) * np.abs(angular_speed)


    goal_reached = abs(angle_diff) < angle_tolerance
    angular_z    = target_theta_speed

    return goal_reached, angular_z

def execute_translation(des_x, des_y, current_x, current_y, current_yaw, distance_tolerance, max_linear_speed, min_linear_speed):

    current_distance_to_goal = np.sqrt((des_x - current_x) ** 2 + (des_y - current_y) ** 2)

    vec1 = np.array([des_x - current_x, des_y - current_y])    
    vec1 = vec1/np.linalg.norm(vec1)
    vec2 = np.array([np.cos(current_yaw), np.sin(current_yaw)])
    vec2 = vec2/np.linalg.norm(vec2)

    linear_speed = np.exp(np.abs(current_distance_to_goal)*2-1) * np.abs(max_linear_speed)
    linear_speed = max(min(linear_speed, max_linear_speed), min_linear_speed)

    target_x_speed = 0 if current_distance_to_goal <= distance_tolerance else np.sign(vec1[0]*vec2[0] + vec1[1]*vec2[1]) * linear_speed

    goal_reached = current_distance_to_goal <= distance_tolerance  
    linear_x     = target_x_speed    

    return goal_reached, linear_x

def execute_translation_distance(distance, start_x, start_y, current_x, current_y, distance_tolerance, max_linear_speed, min_linear_speed):

    current_distance_to_goal = np.abs(distance) - np.sqrt((start_x - current_x) ** 2 + (start_y - current_y) ** 2)

    linear_speed = np.exp(np.abs(current_distance_to_goal)*2-1) * np.abs(max_linear_speed)
    linear_speed = max(min(linear_speed, max_linear_speed), min_linear_speed)

    target_x_speed = 0 if current_distance_to_goal <= distance_tolerance else np.sign(distance) * linear_speed
    goal_reached = current_distance_to_goal <= distance_tolerance
        
    return goal_reached, target_x_speed 