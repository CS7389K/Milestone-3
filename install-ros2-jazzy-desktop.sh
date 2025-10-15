#!/bin/sh

# Compiled script from various guides...
# - ROS2 Jazzy Install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# - Project:
#   - Turtlebot3: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
#   - Dependencies: https://gazebosim.org/docs/harmonic/install_ubuntu/

# Ensure locale is UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable required repositories
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install development tools
sudo apt update && sudo apt install ros-dev-tools

# Install ROS2
sudo apt update
sudo apt upgrade
# Desktop
sudo apt install ros-jazzy-desktop 
# Bare bones
# sudo apt install ros-jazzy-ros-base

# Activate environment
source /opt/ros/jazzy/setup.bash

# Project dependencies
sudo apt-get install -y ros-jazzy-ros-gz
sudo apt install -y ros-jazzy-cartographer \ 
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gripper-controllers \
  ros-jazzy-moveit*

# TurtleBot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
sudo apt install python3-colcon-common-extensions
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc


