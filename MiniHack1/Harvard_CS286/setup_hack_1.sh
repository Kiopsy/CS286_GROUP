#!/bin/bash
sudo apt install -y ros-${ROS_DISTRO}-turtlebot3 ros-${ROS_DISTRO}-turtlebot3-msgs ros-${ROS_DISTRO}-turtlebot3-example ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-dynamixel-sdk
mkdir -p ~/cs286_hack_ws/src
cp -r cs286_mini_hack_1 ~/cs286_hack_ws/src
cd ~/cs286_hack_ws/src
git clone -b ${ROS_DISTRO}-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/cs286_hack_ws && catkin build &&
source ~/cs286_hack_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
echo "source ~/cs286_hack_ws/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
