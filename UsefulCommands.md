run docker image:
docker run -dit --name ros_sim \
  --privileged \
  --network host \
  --ipc host \
  --device /dev/video0:/dev/video0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v /home/oussama/Documents/GitHub/VeniVidiVici/articubot_one:/root/ros_ws/src/articubot_one \
  -v /home/oussama/Documents/GitHub/VeniVidiVici/config:/site_config \
  -v /home/oussama/Documents/GitHub/VeniVidiVici/ball_tracker:/root/ros_ws/src/ball_tracker \
  -v /home/oussama/Documents/GitHub/VeniVidiVici/veni_vidi_vici_bot_one:/root/ros_ws/src/veni_vidi_vici_bot_one \
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:rosready \
  bash



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


run sim of robot
ros2 launch articubot_one launch_sim.launch.py

keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard


ros2 launch articubot_one launch_sim.launch.py world:=./src/articubot_one/worlds/arena_modified.world use_ros2_control:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

ros2 launch slam_toolbox online_async_launch.py params_files:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim:=true

ros2 launch ball_tracker ball_tracker.launch.py   params_file:=src/articubot_one/config/ball_tracker_params_sim.yaml   sim_mode:=true

rviz2 -d /site_config/rviz_conf_laser_img.rviz


ros2 launch veni_vidi_vici_bot_one VVV_launch_sim.launch.py activate_nav:=true activate_loc:=true activate_sm:=true activate_cam:=true use_ros2_control:=true 

ros2 topic pub /start_signal std_msgs/msg/Empty '{}'

source /root/ros_ws/install/setup.bash



rviz2 setup:
fixed frame: map
add tf
add robot model topic: /roboot_description
add laser scan topic: /scan
add map topic: /map
