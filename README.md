# willi
A weeled robot with ROS2 Humble and Ubuntu 22 LTS.

## ROS2 Installation
[Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Additional joystick related packages
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

## Creating workspace
```
mkdir willi_ws
cd willi_ws
colcon build --symlink-install
git clone git@github.com:dkoslowski/willi.git src
```
## Ubuntu tweaks

### Disable services causing 2 min delay at startup
These ubuntu services want the _systemd-networkd-wait-online.service_ which runs into 2 minutes timeout delaying the stratup. Just disable them.
```
sudo systemctl dislable cloud-config open-iscsi cloud-final iscsid
sudo systemctl daemon-reload
```