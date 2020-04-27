# tb3-ros

## Intro

This is a docker package to allow you to run ROS and related applications through your browser. The following is installed:

* ROS Melodic
* Turtlbot3 packages
* prrexamples: examples from the book Programming Robotis with Ros
* Gen5: Assignments for this generation of Cosi119a

## Instructions

### Installation

* Install on your computer by cloning this github repo
* Currently supported on mac with some testing 
* Windows and Linux have had no testing


### Launching it
* cd to the directory
* Controlling the container with `make start` and `make stop`

### Accessing it
* Browser: http://0.0.0.0:6080/vnc.html
* Click on desktop and get a tiny menu. Click "terminal"
* In that terminal do `source setup.bash`
* Now click on desktop and get a second terminal with a legible font
* You should now be able to do all your normal ROS commands such as roscore, rostopic, launch etc.
* 


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
