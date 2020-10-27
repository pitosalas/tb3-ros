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

### Installation

#### Mac Install
* Install Docker for Mac. See [instructions](https://hub.docker.com/editions/community/docker-ce-desktop-mac/)
* Clone this repo
  ```bash
  git clone https://github.com/pitosalas/tb3-ros.git
  ```

#### Windows Install
**Windows has had limited testing!!**

* Install Docker for Windows. See [instructions](https://hub.docker.com/editions/community/docker-ce-desktop-windows/)
* Clone this repo
* Once Docker is installed, go to `Settings` -> `Resources` -> `File Sharing`
  * Check the "+" sign and add the directory of this repo, i.e `C:\Users\Robot\Documents\tb3-ros`

#### Linux Install
* You need both `docker` and `docker-compose` installed
* Clone this repo

### Launching it

#### Mac/Linux Launch
* Start the container
  ```bash
  make start
  ```
* Stop the container
  ```bash
  make stop
  ```

#### Windows Launch
*Using Powershell*
* Start the container
  ```powershell
  .\win-make start
  ```
* Stop the container!
  ```powershell
  .\win-make stop
  ```

### Accessing the virtual desktop
* Browser: http://0.0.0.0:80
* Click on desktop and get a tiny menu. Click "terminal"
* In that terminal do `source setup.bash`

### Accessing a virtual instance of VSCode for coding

* VSCode is a very popular text editor
* You can get to it with http://0.0.0.0:8080
* If it asks for a password, use `dev@ros`

### Setup SSH for remote access
* To enable ssh, first login using virtual desktop/VScode
* Open up the terminal
* Run `sudo passwd root` to setup a password
* To SSH in: `ssh root@host -p 2222`, type in the previous password

## Playing around

* If you don't know much about ROS but want to see what it's like, try these commands
* Open a terminal window and do: `roscore`
* Open a different terminal window and do: `roslaunch turtlebot3_fake turtlebot3_fake.launch`
* Open a different terminal window and do: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

## Access IDE
http://0.0.0.0:8080
password is: `dev@ros`

## Access linux to run graphical programs
http://0.0.0.0:80
password is: `dev@ros`

## Fixing the REST error

```bash
sed -i -e 's/https:\/\/api.ignitionfuel.org/https:\/\/api.ignitionrobotics.org/g' ~/.ignition/fuel/config.yaml
```

## Rebuild the image

*Only used when developing this image.* 

If you want want the latest version of `tb3-ros`, do a `docker pull cosi119/tb3-ros:v2` instead.

```bash
make build
```
