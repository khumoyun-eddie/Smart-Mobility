# ROS 2 Warehouse Automation

The ROS 2 Warehouse Automation project is a robotic system designed for automating warehouse operations. This system includes a central server for task coordination, a robot with a robotic arm for item handling, and a communication protocol using ROS 2 actions.

![Demo](demo.gif)

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)

## Features

- Autonomously manage and optimize warehouse operations.
- Real-time task assignment and monitoring.
- Task execution by a robotic arm with feedback to the central server.
- Customizable and extensible for various warehouse automation tasks.

## Prerequisites

Before you begin, ensure you have met the following requirements:

- ROS 2 Foxy Fitzroy or later: Make sure you have ROS 2 installed. For installation instructions, please visit [ROS 2 Installation Guide](https://index.ros.org/doc/ros2/Installation/).

## Installation

1. Clone the repository to your ROS 2 workspace:

   shell
   cd <your_ros2_ws>/src
   git clone https://github.com/yourusername/warehouse_automation.git
   

2. Build the workspace:

   shell
   colcon build --symlink-install
   

3. Source the workspace:

   shell
   source <your_ros2_ws>/install/setup.bash
   

## Usage

1. Launch the central server:

   shell
   ros2 launch warehouse_automation central_server.launch.py
   

2. Start the robot node (adjust the parameters according to your robot):

   shell
   ros2 run warehouse_automation robot_node --ros-args -p robot_type:=your_robot
   

3. Execute a warehouse task using the action client (customize the task as needed):

   shell
   ros2 run warehouse_automation action_client --ros-args -p task_type:=1 -p item_id:=item123