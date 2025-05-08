run docker image:
sudo docker run --rm --privileged -it --net=host -v /dev/bus/usb:/dev/bus/usb leshrimpkiller/vvv:latest bash

build docker image:
docker build -t leshrimpkiller/vvv .

source:
source install/setup.bash

launch robot:
ros2 launch diffdrive_arduino diffbot.launch.py

list container:
docker container ls

execute terminal in same container
docker exec -it <CONTAINER> bash

run teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

Put code on Arduino
acompile ROSArduinoBridge/ && aupload ROSArduinoBridge/

Controlling motors through pyserial
pyserial-miniterm -e /dev/ttyACM0 57600