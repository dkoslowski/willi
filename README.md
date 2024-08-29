# willi
A 2-wheel robot with ROS2 Jazzy and Ubuntu 24 LTS.

## Configuration

### Controlling host
- Plain PC/Laptop runnung Ubuntu Desktop and ROS2 installed
- Xbox joystick
The joystick can be attached to the robot too. In ths case the ROS "joy_node" must run on the robot.

### Robot
- Raspberry Pi 3B running Ububti Server and ROS2
- Adafruit Motor HAT
- LiPo HAT for powering the RPi
- HC-SR04 ultrasonic distance sensor 
- 2x6V DC brushed motors (with wheels ;)
- 5xAA batteries for powering the motors

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
```
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
All custom ROS stuff will live in the $WS directory
```
WS=<some_directory>
mkdir -p $WS/src
cd $WS/src
git clone git@github.com:dkoslowski/willi.git
```

### Build ROS packages
```
cd $WS
colcon build --symlink-install
```

## Run ROS nodes on the robot
TBD

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
- ["Building your own Open Source robot" by Ubuntu Robotics](https://youtube.com/playlist?list=PL_2PosskAdC25idJVMLOhu-4VAn8OYkQ1)
- [Distance measurement with ultrasonic sensor HC-SR04 (Python)](https://wiki.ros.org/Drivers/Tutorials/DistanceMeasurementWithUltrasonicSensorHC-SR04Python)
- [Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi)
- [CircuitPython helper library provides higher level objects to control motors and servos.](https://pypi.org/project/adafruit-circuitpython-motor/)
- [How to use Raspberry Pi GPIO pins with Ubuntu](https://ubuntu.com/tutorials/gpio-on-raspberry-pi)
- [lgpio Python (local)](https://abyz.me.uk/lg/py_lgpio.html)
