#!/bin/sh

# C++ experiment code
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Python experiment code
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener