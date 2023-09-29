#!/bin/sh

# Managing ROS2 dependencies with rosdep
sudo apt-get install python3-rosdep

# To initialize rosdep
sudo rosdep init
rosdep update

# This is run over a workspace with many packages in a single call to install all dependencies
rosdep install --from-paths src -y --ignore-src

# Creating an action
# Set up a workspace and create a package named action_tutorials_interfaces:
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

# Create an action directory in our ROS 2 package action_tutorials_interfaces
cd action_tutorials_interfaces
mkdir action

# Add following code to CMakeLists.txt
#find_package(rosidl_default_generators REQUIRED)
#    rosidl_generate_interfaces(${PROJECT_NAME}
#    "action/Fibonacci.action"
#    )

# We should also add the required dependencies to our package.xml:
#<buildtool_depend>rosidl_default_generators</buildtool_depend>
#<depend>action_msgs</depend>
#<member_of_group>rosidl_interface_packages</member_of_group>

# We should now be able to build the package containing the Fibonacci action definition:
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build

# Writing an action server and client (C++)
# Create a new package for the C++ action server:
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

# Adding in visibility control
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#adding-in-visibility-control

# Go to the top-level of the ros2_ws, and run:
colcon build

# Running the action client
ros2 run action_tutorials_cpp fibonacci_action_client

# Composing multiple nodes in a single process
# To see what components are registered and available in the workspace
ros2 component types

#Run-time composition using ROS services with a publisher and subscriber
ros2 run rclcpp_components component_container
# run following in the second terminal
ros2 component list

# In the second shell load the talker component (see talker source code):
ros2 component load /ComponentManager composition composition::Talker

# Run-time composition using ROS services with a server and client
ros2 run rclcpp_components component_container

# In the second shell run following
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client

# Compile-time composition using ROS services
ros2 run composition manual_composition

# Run-time composition using dlopen
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so


