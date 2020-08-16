FROM johannhaselberger/coros:latest

# BASE IMAGE CUSTOMIZATIONS -----------------------------------------------------------------------

# Use OpenSWR driver for better OpenGL performance
ENV GALLIUM_DRIVER=swr
# Install system dependencies
RUN apt-get update -y
RUN apt-get install -y \
    iproute2 \
    iputils-ping \
    lxterminal

# COURSE SPECIFIC CONFIGURATIONS ------------------------------------------------------------------
WORKDIR /my_ros_data

ENV TB3_MODEL=burger \
    TURTLEBOT3_MODEL=burger \
    HOME=/my_ros_data \
    NAME_CATKIN_WORKSPACE=/my_ros_data/catkin_ws \
    SHELL=/bin/bash

ADD files/setup.sh .
ADD files/.jwmrc /entry/
COPY files/sshd_config /etc/ssh/

RUN echo 'echo "[running .bashrc]"' >> .bashrc
RUN echo "source /my_ros_data/setup.sh" >> .bashrc

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
RUN git clone https://github.com/campusrover/gpg_bran.git

WORKDIR /my_ros_data/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

WORKDIR /my_ros_data

RUN apt -y update
RUN apt -y upgrade
RUN rosdep update

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

WORKDIR /my_ros_data/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

COPY ./entrypoint.sh /entry/
RUN chmod +x /entry/entrypoint.sh
# Override entrypoint of base image
ENTRYPOINT []
CMD ["bash", "-c", "/entry/entrypoint.sh"]
