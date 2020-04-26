echo "[RUN THIS SCRIPT ONLY"
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=burger
export HOME=/my_ros_data
export NAME_CATKIN_WORKSPACE=/my_ros_data/catkinws

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make

cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/campusrover/prrexamples.git
cd ~/catkin_ws
catkin_make

xterm -fa default -fs 13
sudo apt update
sudo apt upgrade
sudo apt install ros-melodic-slam-gmapping
