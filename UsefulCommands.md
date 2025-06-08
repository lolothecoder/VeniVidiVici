run docker image:
docker run -dit --name ros_vvv \
  --privileged \
  --network host \
  --ipc host \
  --device /dev/video0:/dev/video0 \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/triplev/.Xauthority:/root/.Xauthority:ro \
  -v /home/triplev/Documents/VeniVidiVici/veni_vidi_vici_bot_one:/root/ros_ws/src/veni_vidi_vici_bot_one \
  -v /home/triplev/Documents/VeniVidiVici/rplidar_ros2:/root/ros_ws/src/rplidar_ros2 \
  -v /home/triplev/Documents/VeniVidiVici/diffdrive_arduino:/root/ros_ws/src/diffdrive_arduino \
  -v /home/triplev/Documents/VeniVidiVici/ball_tracker:/root/ros_ws/src/ball_tracker \
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:latest bash


execute terminal in same container
docker exec -it ros_vvv bash

build docker image:
docker build -t leshrimpkiller/vvv .

source:
source install/setup.bash

launch robot:
alaunch
ros2 launch veni_vidi_vici_bot_one launch_robot.launch.py 

ros2 launch veni_vidi_vici_bot_one VVV_launch_robot.launch.py activate_loc:=true activate_nav:=true activate_sm:=true

list container:
docker container ls

run teleop:
ateleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

Put code on Arduino
acompile ROSArduinoBridge/ && aupload ROSArduinoBridge/

Controlling motors through pyserial
pyserial-miniterm -e /dev/ttyACM0 57600

create new bash
anewbash
sudo docker exec -it $(sudo docker ps -aqf "ancestor=leshrimpkiller/vvv") bash

Launch lidar
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard

ros2 launch rplidar_ros rplidar.launch.py frame_id:=laser_frame

Run Rviz docker:

ssh -X triplev@172.21.68.240

xhost +local:root  # Allow local root user to access X server

export XAUTHORITY=/home/triplev/.Xauthority

sudo docker run --rm -it --privileged --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/triplev/.Xauthority:/root/.Xauthority:ro \
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:latest bash

echo $DISPLAY       
echo $XAUTHORITY 

rviz2

ros2 launch slam_toolbox online_async_launch.py params_file:=./src/veni_vidi_vici_bot_one/config/mapper_params_online_async.yaml use_sim_time:=false

ros2 launch veni_vidi_vici_bot_one VVV_launch_robot.launch.py activate_sm:=true use_ros2_control:=true activate_nav:=true activate_loc:=true

source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 launch veni_vidi_vici_bot_one VVV_launch_robot.launch.py activate_nav:=true activate_loc:=true activate_sm:=true use_ros2_control:=true 

ros2 topic pub /start_signal std_msgs/msg/Empty '{}'

