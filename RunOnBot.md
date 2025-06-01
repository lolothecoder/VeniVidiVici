Plug in therobot using either an alim or battery

With alim use adapter

With battery take out the adapter and plug in the white JST connector (RED N RED PLEASE). Then plug in the power. Press the reset button on the powerboard. Flip the switch on. If it doesn't work play around with the switch (On and off). MAKE SURE the battery has more than 10.5 V left in it.

Pull new docker image:
docker pull leshrimpkiller/vvv

login info:
ssh: lolon@172.21.69.243
pswd: milotamario64

If you want to use rviz2 or any GUI apps:
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

!!MAKE SURE YOU RUN THE LAUNCH FILE THAT LAUNCHES GUI APPS LAST!!

Open up a bunch of terminals with either 
- anewbash
- docker exec -it <CONTAINER> bash

Source terminal

FIRST run the robot launch file with : 
ros2 launch veni_vidi_vici_bot_one launch_robot.launch.py 
