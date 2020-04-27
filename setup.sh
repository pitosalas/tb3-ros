echo "[running setup.sh]"
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
export HOME=/my_ros_data
export NAME_CATKIN_WORKSPACE=/my_ros_data/catkin_ws

source /opt/ros/melodic/setup.bash
source /my_ros_data/catkin_ws/devel/setup.bash

alias sb='source ~/.bashrc'
alias gs='git status'
alias cw='cd $NAME_CATKIN_WORKSPACE'
alias cs='cd $NAME_CATKIN_WORKSPACE/src'
alias cm='cd $NAME_CATKIN_WORKSPACE && catkin_make'
xrdb -merge ~/.Xresources