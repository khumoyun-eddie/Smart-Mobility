#!/bin/sh

# run this command on every new shell you open to have access to the ROS 2 commands
source /opt/ros/humble/setup.bash

# or instead of runnig above command on every new shell, we can use following
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Several environment variables necessary for operating ROS 2, to check them 
printenv | grep -i ROS

#
export ROS_DOMAIN_ID=100

#
echo "export ROS_DOMAIN_ID=100" >> ~/.bashrc

#
export ROS_LOCALHOST_ONLY=1

#
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc


#
sudo apt update

sudo apt install ros-humble-turtlesim

#
ros2 pkg executables turtlesim

#
ros2 run turtlesim turtlesim_node

#
ros2 run turtlesim turtle_teleop_key

#
sudo apt update

sudo apt install ~nros-humble-rqt*

# Understanding nodes
# To run turtlesim
ros2 run turtlesim turtlesim_node

# in the new terminal
ros2 node list

# Assigning new name to /turtlesim node
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# To access more information about node
ros2 node info /my_turtle

# Understanding topics
# Running turtlesim
ros2 run turtlesim turtlesim_node

#
ros2 run turtlesim turtle_teleop_key

# use rqt_graph to visualize the changing nodes and topics
rqt_graph

# a list of all the topics currently active in the system
ros2 topic list

# same as above but with active topics inside brackets
ros2 topic list -t

# To see the data being published on a topic
ros2 topic echo /turtle1/cmd_vel

# This expresses velocity in free space broken into its linear and angular parts.
ros2 interface show geometry_msgs/msg/Twist

# To get the turtle to keep moving
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Understanding services
# A list of all the services currently active in the system
ros2 service list 

# If empty, the service call sends no data when making a request and receives no data when receiving a response.
ros2 service type /clear

# To see the types of all the active services at the same time
ros2 service list -t

# To find all the services of a specific type. Eg.: Empty
ros2 service find std_srvs/srv/Empty

# To call services from the command line
ros2 interface show turtlesim/srv/Spawn

# To call next turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

# Understanding parameters
# to see the parameters belonging to your nodes
ros2 param list

# to display the type and current value of a parameter
ros2 param get /turtlesim background_g

# to change a parameter's value at runtime
ros2 param set /turtlesim background_r 150

# to view all of a node's current params
ros2 param dump /turtlesim > turtlesim.yaml


# load parameters from a file to a currently running node 
ros2 param load /turtlesim turtlesim.yaml

# Understanding actions
# To see the list of actions a node provides
ros2 node info /turtlesim

# To identify all the actions in the ROS graph
ros2 action list

# To send an action goal
ros2 interface show turtlesim/action/RotateAbsolute

# Understanding rqt_console
# To open rqt_console
ros2 run rqt_console rqt_console

# start turtlesim 
ros2 run turtlesim turtlesim_node

# to produce log messages for rqt_console to display
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

# set the default logger level when you first run the /turtlesim node using remapping. 
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN

# Launching nodes
# To run launch file 
ros2 launch turtlesim multisim.launch.py

# To make turtles drive in opposite directions 
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Recording and playing back data
# For ros2 bag we need to install packages
sudo apt-get install ros-humble-ros2bag \ ros-humble-rosbag2-storage-default-plugins

# Run turtlesim and turtle_teleop_key in 2 terminals
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtlesim_teleop_key

# to see list of your system;s topics
ros2 topic list

# to see the data that /turtle1/cmd_vel
ros2 topic echo /turtle1/cmd_vel

# to record the data published to a topic 
ros2 bag record /turtle1/cmd_vel

# for recording multiple topics
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

# to see details about your recording
ros2 bag info subset



