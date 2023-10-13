## ROS Discovery Server

This tutorial will show you how to use the Fast DDS Discovery Server as a discovery protocol for ROS 2.

### What is the Fast DDS Discovery Server?

The Fast DDS Discovery Server is a centralized discovery mechanism that can be used to reduce discovery-related network traffic and that does not require multicasting capabilities.

### Why use the Fast DDS Discovery Server?

There are several reasons why you might want to use the Fast DDS Discovery Server:

* It can reduce discovery-related network traffic.
* It does not require multicasting capabilities.
* It is easy to use.
* It is reliable.

### How to use the Fast DDS Discovery Server with ROS 2

To use the Fast DDS Discovery Server with ROS 2, you will need to:

1. Install the Fast DDS Discovery Server.
2. Configure ROS 2 to use the Fast DDS Discovery Server.
3. Start the Fast DDS Discovery Server.
4. Start your ROS 2 nodes.

### Step-by-step instructions

1. Install the Fast DDS Discovery Server.

```
# Ubuntu
sudo apt install ros-humble-fastdds-discovery-server

# macOS
brew install ros/humble/fastdds-discovery-server
```

2. Configure ROS 2 to use the Fast DDS Discovery Server.

Create a file called `ros2_environment.sh` in your `.bashrc` file. This file will contain the environment variables that ROS 2 needs to use the Fast DDS Discovery Server.

```
# .bashrc
source /opt/ros/humble/setup.bash

# Set the ROS_DISCOVERY_SERVER environment variable to the address of the Fast DDS Discovery Server.
export ROS_DISCOVERY_SERVER=localhost:5555
```

3. Start the Fast DDS Discovery Server.

```
ros2 launch fastdds_discovery_server fastdds_discovery_server.launch.py
```

4. Start your ROS 2 nodes.

```
ros2 launch my_package my_node.launch.py
```