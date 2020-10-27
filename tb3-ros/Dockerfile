FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

# BASE IMAGE CUSTOMIZATIONS -----------------------------------------------------------------------

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# Install system dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    iproute2 \
    iputils-ping \
    python \
    python-pip \
    python-dev \
    python-mako \
    nano \
    wget \
    make \
    git \
    curl \
    openssh-server

# ROS melodic support
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-melodic-rqt \
    ros-melodic-rqt* \
    ros-melodic-rviz \
    ros-melodic-desktop-full

# Python3 support
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-pip \
    python3-yaml

# Code-server
RUN mkdir /code-server && \
    curl -fL https://github.com/cdr/code-server/releases/download/v3.6.1/code-server-3.6.1-linux-amd64.tar.gz \
    | tar xvz --strip-components=1 -C /code-server

# COURSE SPECIFIC CONFIGURATIONS ------------------------------------------------------------------
WORKDIR /my_ros_data

ENV TB3_MODEL=burger \
    TURTLEBOT3_MODEL=burger \
    HOME=/my_ros_data \
    NAME_CATKIN_WORKSPACE=/my_ros_data/catkin_ws

# Copy needed files
ADD files/rootfs/.bashrc .
ADD files/rootfs/.jwmrc /entry/
COPY files/sshd/sshd_config /etc/ssh/
COPY files/nginx/default /etc/nginx/sites-enabled/
COPY files/supervisor/supervisord.conf /etc/supervisor/conf.d/

RUN mkdir -p /my_ros_data/.config/code-server
COPY files/code-server/config.yaml /my_ros_data/.config/code-server/config.yaml

RUN git clone https://github.com/campusrover/rosutils.git

WORKDIR /my_ros_data/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace"

WORKDIR /my_ros_data/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

WORKDIR /my_ros_data/catkin_ws/src
RUN git clone --recursive --depth=1 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone --recursive --depth=1 https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone --recursive --depth=1 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN git clone --recursive --depth=1 https://github.com/campusrover/prrexamples.git
RUN git clone --recursive --depth=1 https://github.com/campusrover/gen5.git
RUN git clone --recursive --depth=1 https://github.com/MoffKalast/rbx1.git
RUN git clone --recursive --depth=1 https://github.com/campusrover/robot_services.git
RUN git clone --recursive --depth=1 https://github.com/campusrover/gpg_bran4.git
RUN git clone --recursive --depth=1 https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming.git

WORKDIR /my_ros_data

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-melodic-slam-gmapping \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-dwa-local-planner \
    ros-melodic-fiducials \
    ros-melodic-arbotix-* \
    ros-melodic-turtlebot*

RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

RUN pip install redis
RUN pip3 install \
    rospkg \
    catkin_pkg

WORKDIR /my_ros_data/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

COPY ./entrypoint.sh /entry/
RUN chmod +x /entry/entrypoint.sh
# Override entrypoint of base image
ENTRYPOINT []
CMD ["bash", "-c", "/entry/entrypoint.sh"]
