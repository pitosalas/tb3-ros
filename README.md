# tb3-ros

## Intro

This is a docker package to allow you to run ROS and related applications through your browser. The following is installed:

* ROS Melodic
* Turtlbot3 packages
* prrexamples: examples from the book Programming Robotis with Ros
* Gen5: Assignments for this generation of Cosi119a
* Here are some handy commands for later: https://campus-rover.gitbook.io/lab-notebook/ros-tips/handy-commands

## Standalone image

**Tested on Windows, Mac, Ubuntu**

`tb3-ros` can be started as a standalone container.

See [tb3-ros](tb3-ros/README.md).

## Server mode

**Only tested on Ubuntu 18.04**

`tb3-ros` can be started as a server with multiple containers running this image.

See [serverctl](server/README.md).
