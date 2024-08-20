# willi
A weeled robot with ROS2 Jazzy and Ubuntu 24 LTS.

## ROS2 Installation
[Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

## ROS2 Environment
Add to `~/.bashrc`:
```
# ROS 2
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=68
```
## Install colcon
```
sudo apt-get install -y python3-colcon-common-extensions
```

## System tweaks
Add user to the "input" group to allow joystick operations
```
sudo usermod -aG input $USER
```

## Additional packages
Joystick related packages
```
sudo apt-get install -y joystick jstest-gtk evtest
```

## Install pip and creatre python venv
```
sudo apt install pip3
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

## Install Adafruit MotorKit library inside venv
```
pip3 install rpi.gpio adafruit-circuitpython-motorkit
```

---


## Creating workspace
```
mkdir Robots
cd Robots
git clone git@github.com:dkoslowski/willi.git
cd willi
colcon build --symlink-install
```

## Adafruit Motor HAT

Install additional packages
```
sudo apt-get install -y python3-smbus i2c-tools
sudo i2cdetect -y 1

```
Install Adafruit MotorKit library
```
sudo apt install pip3
python3 -m venv .venv
source .venv/bin/activate
pip3 install rpi.gpio adafruit-circuitpython-motorkit
```

## Controlling the robot
### by command publishing
```
ros2 topic pub '/command' 'std_msgs/String' '{data: forward}'
ros2 topic pub '/command' 'std_msgs/String' '{data: backward}'
ros2 topic pub '/command' 'std_msgs/String' '{data: left}'
ros2 topic pub '/command' 'std_msgs/String' '{data: right}'
ros2 topic pub '/command' 'std_msgs/String' '{data: stop}'
```

## Ubuntu tweaks

### (Obsolete) Disable services causing startup delay on RasPi
These ubuntu services want the `systemd-networkd-wait-online.service` which runs into 2 minutes timeout delaying the startup. Just disable them.
```
sudo systemctl dislable cloud-config open-iscsi cloud-final iscsid
sudo systemctl daemon-reload
```

## Links
- ["Building your own Open Source robot" by Ubuntu Robotics](https://youtube.com/playlist?list=PL_2PosskAdC25idJVMLOhu-4VAn8OYkQ1)
- [Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi)
- [CircuitPython helper library provides higher level objects to control motors and servos.](https://pypi.org/project/adafruit-circuitpython-motor/)
- [How to use Raspberry Pi GPIO pins with Ubuntu](https://ubuntu.com/tutorials/gpio-on-raspberry-pi)
