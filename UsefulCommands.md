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
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:latest bash


build docker image:
docker build -t leshrimpkiller/vvv .

source:
source install/setup.bash

launch robot:
alaunch
ros2 launch veni_vidi_vici_bot_one launch_robot.launch.py 

list container:
docker container ls

execute terminal in same container
docker exec -it ros_vvv bash

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

Run Rviz docker:

ssh -X lolon@172.21.69.243

xhost +local:root  # Allow local root user to access X server

export XAUTHORITY=/home/lolon/.Xauthority

sudo docker run --rm -it --privileged --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/lolon/.Xauthority:/root/.Xauthority:ro \
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:latest bash

echo $DISPLAY       
echo $XAUTHORITY 

rviz2

ros2 launch slam_toolbox online_async_launch.py params_file:=./src/veni_vidi_vici_bot_one/config/mapper_params_online_async.yaml use_sim_time:=false

source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=5
