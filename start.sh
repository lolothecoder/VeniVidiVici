#!/usr/bin/env bash
set -eu

docker stop ros_vvv
docker rm ros_vvv
# 1) start container in background
docker rm -f ros_vvv 2>/dev/null || true
docker run -d --name ros_vvv \
  --privileged \
  --network host \
  --ipc host \
  --device /dev/video0:/dev/video0 \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.Xauthority":/root/.Xauthority:ro \
  -v "$HOME/Documents/VeniVidiVici":/root/ros_ws/src/VeniVidiVici \
  -v /dev/bus/usb:/dev/bus/usb \
  leshrimpkiller/vvv:latest \
  tail -f /dev/null

# 2) run build & launch non-interactively
docker exec ros_vvv bash -lc "
  set -ex
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
  source /root/ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=5
  ros2 launch veni_vidi_vici_bot_one VVV_launch_robot.launch.py \
    activate_nav:=true activate_loc:=true activate_sm:=true use_ros2_control:=true
"

# 3) finally give yourself an interactive prompt
docker exec -it ros_vvv bash
