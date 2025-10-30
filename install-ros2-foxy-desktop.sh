#!/bin/sh

## Compiled script from various guides...
# - ROS2 foxy:  https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
# - Turtlebot3: https://web.archive.org/web/20240309202534/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator

## Ensure locale is UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

## Enable required repositories
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

## Install Dependencies
# ROS2 - Desktop
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop python3-argcomplete -y
# ROS2 - Bare Bones
# sudo apt install ros-foxy-ros-base python3-argcomplete -y
# ROS2 - Devtools
sudo apt install ros-dev-tools -y
# Activate ROS2 Environment
source /opt/ros/foxy/setup.bash
# TurtleBot3 Dependencies
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-foxy-gazebo-* \
  ros-foxy-cartographer \
  ros-foxy-cartographer-ros \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-dynamixel-sdk \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-gripper-controllers \
  ros-foxy-moveit
# TurtleBot3 Modules
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b foxy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b foxy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b foxy https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
cd ~/turtlebot3_ws
colcon build --symlink-install

## Environment
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc
