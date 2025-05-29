FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y nano \
    python3-pip \
    python3-serial \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-twist-mux \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

COPY config/ /site_config/

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
    && apt-get install sudo -y \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

WORKDIR /root/ros_ws/src


RUN git clone https://github.com/joshnewans/serial.git
ADD https://www.google.com /time.now
WORKDIR /root/ros_ws/src
RUN rm -rf VeniVidiVici &&\
    git clone https://github.com/lolothecoder/VeniVidiVici.git

WORKDIR /root/ros_ws/src
RUN git clone https://github.com/babakhani/rplidar_ros2.git

WORKDIR /root/ros_ws/src/serial
RUN git checkout 2121a37eaa1aff8ca62badc0ac8f43b87169d706 && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; make" && \
    make install

RUN echo 'alias ateleop="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped"' >> ~/.bashrc
RUN echo 'alias alaunch="ros2 launch diffdrive_arduino diffbot.launch.py"' >> ~/.bashrc

WORKDIR /root/ros_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; \
    colcon build --symlink-install; \
    source install/setup.bash"

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

CMD ["bash"]