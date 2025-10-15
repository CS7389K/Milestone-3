#!/bin/sh

## Compiled script from various guides...
# - ROS2 Humble:     https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# - Turtlebot3: 
#   - Quick Start:   https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
#   - Simulation:    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
#   - Manipulation:  https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/

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
sudo apt install ros-humble-desktop -y
# ROS2 - Bare Bones
# sudo apt install ros-humble-ros-base
# ROS2 - Devtools
sudo apt install ros-dev-tools -y
# Activate ROS2 Environment (change to fit your shell)
# source /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.sh
# source /opt/ros/humble/setup.zsh
# TurtleBot3 Dependencies
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-gazebo-* \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-dynamixel-sdk \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gripper-controllers \
  ros-humble-moveit*
# TurtleBot3 Modules
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble  https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble  https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble  https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble  https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble  https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
cd ~/turtlebot3_ws
colcon build --symlink-install

## Environment
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc


