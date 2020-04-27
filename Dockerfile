FROM johannhaselberger/coros:latest

WORKDIR /my_ros_data

ADD setup.sh .
ADD .Xresources .

ENV TB3_MODEL=burger \
    TURTLEBOT3_MODEL=burger \
    HOME=/my_ros_data \
    NAME_CATKIN_WORKSPACE=/my_ros_data/catkinws
RUN echo "source /my_ros_data/setup.sh" >> .bashrc

RUN mkdir -p catkin_ws/src

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

WORKDIR /my_ros_data/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

WORKDIR /my_ros_data

RUN sudo apt update
RUN apt -y upgrade
RUN apt -y install ros-melodic-slam-gmapping
