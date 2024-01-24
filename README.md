# willi
A weeled robot with ROS2 Humble and Ubuntu 22 LTS.

## ROS2 Installation
[Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Additional packages and tweaks
```
sudo apt install joystick jstest-gtk evtest
sudo usermod -aG input $USER
```

## ROS2 Environment
Add to `~/.bashrc`:
```
# ROS 2
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=68
```