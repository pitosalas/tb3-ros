FROM cosi119/coros:latest

# BASE IMAGE CUSTOMIZATIONS -----------------------------------------------------------------------

# Use OpenSWR driver for better OpenGL performance
ENV GALLIUM_DRIVER=swr \
    SHELL=/bin/bash

# Install system dependencies
RUN apt-get update -y
RUN apt-get install -y \
    iproute2 \
    iputils-ping \
    lxterminal \
    locales

# Fix locale issue
RUN locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Python3 support
RUN apt-get install -y \
    python3-pip \
    python3-yaml

# COURSE SPECIFIC CONFIGURATIONS ------------------------------------------------------------------
WORKDIR /my_ros_data

ENV TB3_MODEL=burger \
    TURTLEBOT3_MODEL=burger \
    HOME=/my_ros_data \
    NAME_CATKIN_WORKSPACE=/my_ros_data/catkin_ws

# Copy needed files
ADD files/setup.sh .
ADD files/.jwmrc /entry/
COPY files/sshd_config /etc/ssh/

RUN echo 'echo "[running .bashrc]"' >> .bashrc
RUN echo "source /my_ros_data/setup.sh" >> .bashrc
RUN git clone https://github.com/campusrover/rosutils.git

WORKDIR /my_ros_data/catkin_ws/src
RUN source /opt/ros/melodic/setup.bash && catkin_init_workspace

WORKDIR /my_ros_data/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

WORKDIR /my_ros_data/catkin_ws/src
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN git clone https://github.com/campusrover/prrexamples.git
RUN git clone https://github.com/campusrover/gen5.git
RUN git clone https://github.com/MoffKalast/rbx1.git
RUN git clone https://github.com/campusrover/robot_services.git
RUN git clone https://github.com/campusrover/gpg_bran4.git
RUN git clone https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming.git

WORKDIR /my_ros_data

RUN apt -y update
RUN apt -y upgrade

RUN apt-get -y install \
    ros-melodic-slam-gmapping \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-dwa-local-planner \
    ros-melodic-fiducials \
    ros-melodic-arbotix-* \
    ros-melodic-turtlebot*

RUN rosdep update

RUN pip install redis
RUN pip3 install \
    rospkg \
    catkin_pkg

WORKDIR /my_ros_data/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

COPY ./entrypoint.sh /entry/
RUN chmod +x /entry/entrypoint.sh
# Override entrypoint of base image
ENTRYPOINT []
CMD ["bash", "-c", "/entry/entrypoint.sh"]
