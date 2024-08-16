# willi
A weeled robot with ROS2 Jazzy and Ubuntu 24 LTS.

## ROS2 Installation
[Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

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

## Install colcon
```
sudo apt install python3-colcon-common-extensions
```

## Creating workspace
```
mkdir willi_ws
cd willi_ws
colcon build --symlink-install
git clone git@github.com:dkoslowski/willi.git src
```
## Ubuntu tweaks

### Disable services causing startup delay on RasPi (obsolete?)
These ubuntu services want the `systemd-networkd-wait-online.service` which runs into 2 minutes timeout delaying the startup. Just disable them.
```
sudo systemctl dislable cloud-config open-iscsi cloud-final iscsid
sudo systemctl daemon-reload
```
