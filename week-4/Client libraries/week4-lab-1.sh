#!/bin/sh

# Using colcon to build packages
sudo apt install python3-colcon-common-extensions

# Create a directory to contain our workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone the examples repo into the src directory
git clone https://github.com/ros2/examples src/examples -b humble

# create an underlay in ros2_ws
colcon build --symlink-install
