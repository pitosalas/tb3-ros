export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
export HOME=/my_ros_data
export NAME_CATKIN_WORKSPACE=/my_ros_data/catkin/ws

source /opt/ros/melodic/setup.bash
source /my_ros_data/catkin_ws/devel/setup.bash

alias sb='source ~/.bashrc'
alias gs='git status'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/$name_catkin_workspace/src'
alias cm='cd ~/$name_catkin_workspace && catkin_make'
