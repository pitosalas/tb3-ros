# tb3-ros v2

Standalone docker image for running `tb3-ros`.

## Specifications

* Ubuntu 18.04
* ROS Melodic
* lxde window manager

Services:
* `80` NoVNC
* `5900` VNC
* `8080` VSCode
* `2222` SSH

## Instructions

See [campusrover/clouddesktop-docker](https://github.com/campusrover/clouddesktop-docker).

## Build the image

*Only used when developing this image.*

```bash
make build
```
