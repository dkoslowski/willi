# willi
A simply 2-wheel robot. Based on ["Building your own Open Source robot"](https://youtube.com/playlist?list=PL_2PosskAdC25idJVMLOhu-4VAn8OYkQ1) YouTube presentation by Sid Faber from Ubuntu Robotics.

## Features
- Distributed controlling system (robot/host) connected via WiFi 
- Runs on ROS2 Jazzy and Ubuntu 24 LTS
- Steering by console, graphical tool or gamepad
- Collision avoidance (forward direction only)

The controlling host is optional. You can run all controlling nodes on the robot with attached (wireless) gamepad. In this case you'll still need a PC to ssh to the robot and configure the X11 if you want to see the graphic output.

Collision avoidanse is optional too. In this case you don't need to run the appropriate sensor node. Actually, if there is no sensor, this node won't run at all ;)

## Configuration

### Controlling host
- Plain PC/Laptop runnung Ubuntu Desktop and ROS2 installed
- Xbox gamepad
The (wireless) gamepad can be attached to the robot too. In ths case the corresponding ROS "joy_node" must run on the robot.

### Robot
- Raspberry Pi 3B running Ububti Server and ROS2
- Adafruit Motor HAT
- LiPo HAT for powering the RPi
- HC-SR04 ultrasonic distance sensor 
- 2x6V DC brushed motors (with wheels ;)
- 5xAA batteries for powering the motors

# Bootstrapping
## ROS2 Installation
### ROS2
[Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### ROS2 Environment
Add to `~/.bashrc`:
```
# ROS 2
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=68

# ROS workspace, all custom stuff will live in this directory
WS=~/willi_ws

```
To make changes take effect, do a logout/login cycle

## colcon extentions
```
sudo apt-get install -y python3-colcon-common-extensions
```

## I/O support
NB: appliable either on the robot or on the controlling host

### RPi.gpio & lgpio libraries (robot)
```
sudo apt-get install -y python3-rpi.gpio python3-lgpio
```

### Adafruit MotorKit (robot) 
```
sudo apt install python3-pip
sudo pip3 install --break-system-packages adafruit-circuitpython-motorkit
```

### System tweaks (controlling host)
Add user to the "input" group to allow joystick operations
```
sudo usermod -aG input $USER
```
## Custom software
### Create workspace
```
mkdir -p $WS/src
cd $WS/src
git clone git@github.com:dkoslowski/willi.git
```
# Build
## Build custom ROS packages
```
cd $WS
colcon build --symlink-install
```
## Activate custom ROS overlay
```
source $WS/install/setup.bash
```

# Run
## Build the ROS package
```
cd $WS
colcon build --symlink-install
```

## Start ROS nodes
### Sensor node (on robot)
```
source $WS/install/setup.bash
ros2 run willi sensor_node
```

### Control node (on robot, new terminal window)
```
source $WS/install/setup.bash
ros2 run willi sensor_node
```

## Controlling the robot
### by direct command publishing
```
ros2 topic pub '/command' 'std_msgs/String' '{data: forward}'
ros2 topic pub '/command' 'std_msgs/String' '{data: backward}'
ros2 topic pub '/command' 'std_msgs/String' '{data: left}'
ros2 topic pub '/command' 'std_msgs/String' '{data: right}'
ros2 topic pub '/command' 'std_msgs/String' '{data: stop}'

ros2 topic echo /command
```
### by keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic echo /cmd_vel
```

### by rqt plugin
```
sudo apt install ros-jazzy-rqt-robot-steering
ros2 run rqt_robot_steering rqt_robot_steering
ros2 topic echo /cmd_vel
```

### by joystik
```
ros2 run joy joy_node
ros2 topic echo /joy
```

## Links
- ["Building your own Open Source robot"](https://youtube.com/playlist?list=PL_2PosskAdC25idJVMLOhu-4VAn8OYkQ1) by Sid Faber
- [Distance measurement with ultrasonic sensor HC-SR04 (Python)](https://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Python)
- [Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi)
- [CircuitPython helper library provides higher level objects to control motors and servos.](https://pypi.org/project/adafruit-circuitpython-motor/)
- [How to use Raspberry Pi GPIO pins with Ubuntu](https://ubuntu.com/tutorials/gpio-on-raspberry-pi)
- [lgpio Python (local)](https://abyz.me.uk/lg/py_lgpio.html)
