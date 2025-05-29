run docker image:
sudo docker run --rm --privileged -it --net=host -v /dev/bus/usb:/dev/bus/usb  leshrimpkiller/vvv:latest bash

build docker image:
docker build -t leshrimpkiller/vvv .

source:
source install/setup.bash

launch robot:
alaunch
ros2 launch diffdrive_arduino diffbot.launch.py

list container:
docker container ls

execute terminal in same container
docker exec -it <CONTAINER> bash

run teleop:
ateleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

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