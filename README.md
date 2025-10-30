# Milestone 3

In this milestone, you will try operating a physical robot arm and write a ROS 2  program to control the OpenManipulator on the Turtlebot using a keyboard. The program should be similar to turtlebot3_teleop_key, but it will accept a few more keystrokes to control the robot arm. Your program should provide some basic functions, such as setting the arm to some predefined poses and opening/closing the gripper. In addition, your program should read the current configuration of the arm and the gripper and display the information on the screen. Of course, please feel free to add more functionalities to your program. 

## ROS2 Foxy Guides

- [Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Development Environment

### Setup

1. Clone the repository
```sh
git clone https://github.com/andrewscouten/CS7389K_Homework_3.git
cd CS7389K_Homework_3
```

2. ROS2 Foxy requires Ubuntu 20.04, so ensure it's what you're using. If you're using windows, run the following to use WSL:
```sh
install-wsl2-ros2-env.bat
```

3. Install Foxy
```sh
sh install-ros2-foxy-desktop.sh
```

### Building the Project

```sh
sh build.sh
```
