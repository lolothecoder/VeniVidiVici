colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=5
ros2 launch veni_vidi_vici_bot_one VVV_launch_robot.launch.py activate_nav:=true activate_loc:=true activate_sm:=true use_ros2_control:=true 