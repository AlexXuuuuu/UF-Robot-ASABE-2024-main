#!/bin/bash
source /opt/ros/iron/opt/setup.bash
cd ~/ros_ws
export GZ_VERSION=harmonic
ech $GZ_VERSION
colcon build
source ./setup.bash
ros2 launch asabe_2024 arm.launch.py