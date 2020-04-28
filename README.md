# tb3-ros

## Intro

This is a docker package to allow you to run ROS and related applications through your browser. The following is installed:

* ROS Melodic
* Turtlbot3 packages
* prrexamples: examples from the book Programming Robotis with Ros
* Gen5: Assignments for this generation of Cosi119a
* Here are some handy commands for later: https://campus-rover.gitbook.io/lab-notebook/ros-tips/handy-commands

## Instructions

### Installation

* Install on your computer by cloning this github repo
* Currently supported on mac with some testing 
* Windows and Linux have had no testing

#### Mac Install
* Install docker
* Clone this repo to your computer
roslaunch turtlebot3_fake turtlebot3_fake.launch
#### Windows Install
* Install "Ubuntu on Windows"
* Install Docker
* Clone this repo

### Launching it
* cd to the directory
* Controlling the container with `make start` and `make stop`

### Accessing the virtual desktop
* Browser: http://0.0.0.0:6080/vnc.html
* Click on desktop and get a tiny menu. Click "terminal"
* In that terminal do `source setup.bash`
* Now click on desktop and get a second terminal with a legible font

### Accessing a virtual instance of VSCode for coding

* VSCode is a very popular text editor
* You can get to it with http://0.0.0.0:80
* If it asks for a password, use dev@ros

## Playing around

* If you don't know much about ROS but want to see what it's like, try these commands
* Open one terminal window and do `roslaunch turtlebot3_fake turtlebot3_fake.launch`
* Open a diffetent terminal and do: `roslaunch turtlebot3_teleop turtlebot3_teleop_key`


## Working with the container

### Start container

```bash
make start
```

### Stop container

```bash
make stop
```

### Fix any issues introduced by force-stop

```bash
make fix
```

### Rebuild the image

```bash
make build
```

## Access IDE
http://0.0.0.0:80
password is: dev@ros

## Access linux to run graphical programs
http://0.0.0.0:6080/vnc.html
