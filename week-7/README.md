# Warehouse Automation Robot with ROS2

![Cover Gazebo Robot](cover_image.jpg)

## Table of Contents

- [Introduction](#introduction)
- [Real-World Applications](#real-world-applications)
- [Prerequisites](#prerequisites)
- [Build the Warehouse Robot](#build-the-warehouse-robot)
- [Integrate ROS 2 and Gazebo](#integrate-ros-2-and-gazebo)
- [Build a Warehouse](#build-a-warehouse)

## Introduction

Welcome to the "Cover Gazebo Robot" project! In this tutorial, we will explore the process of creating an autonomous mobile robot from scratch using Gazebo. This project provides a step-by-step guide to building a warehouse robot, including creating the environment, integrating ROS 2, and controlling the robot using velocity commands.

### What You Will Build

The robot we are creating is an autonomous differential drive mobile warehouse robot. We will start from the ground up, building the entire SDF file (Simulation Description Format) for the robot. Our simulated robot will resemble the one used by Fetch Robotics, a mobile robotics company based in Silicon Valley, California.

![Fetch Warehouse Robot](fetch-warehouse.jpg)
*Credit: Fetch Robotics*

## Real-World Applications

Mobile warehouse robots have extensive real-world applications, such as those used by Amazon to transport shelves in fulfillment centers, ensuring fast and efficient order fulfillment. Additionally, simulating robots before physical implementation is essential for testing algorithms, as it helps prevent costly mistakes in real-world scenarios.

## Prerequisites

Before you begin, make sure you have the following prerequisites:

- ROS 2 Installed: Ensure you have ROS 2 installed on your Ubuntu Linux system. This tutorial uses ROS 2 Foxy. If you haven't installed ROS 2, you can follow the [ROS 2 Installation Guide](https://index.ros.org/doc/ros2/Installation/).

- Basic Gazebo Knowledge: Familiarity with creating a basic mobile robot in Gazebo will be helpful for understanding the steps.

## Build the Warehouse Robot

### Create Model.config

1. Create a folder for the model:

   bash
   mkdir -p ~/.gazebo/models/mobile_warehouse_robot
   

2. Create a model configuration file:

   bash
   gedit ~/.gazebo/models/mobile_warehouse_robot/model.config
   

   Modify the model.config file to include the robot's name, version, author information, and a brief description.

3. Save the file and close it.

### Download the Mesh Files

Mesh files improve the robot's visual appearance. Download the warehouse robot and Hokuyo Laser Range Finder mesh files using the following commands:

bash
cd ~/.gazebo/models
wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/warehouse_robot/


Now, download the Hokuyo Laser Range Finder mesh file:

bash
cd ~/.gazebo/models
wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/hokuyo/


### Create Model.sdf

1. Create an SDF file to define the robot's structure:

   bash
   gedit ~/.gazebo/models/mobile_warehouse_robot/model.sdf
   

   Copy and paste the SDF file contents, defining the robot with three wheels and a mounted laser range finder.

2. Save the file and close it.

### Test Your Robot

1. Run Gazebo to visualize the robot:

   bash
   gazebo
   

2. In Gazebo, click the "Insert" tab and select "Mobile Warehouse Robot." Place the robot within the environment.

3. Close Gazebo by typing CTRL + C in the terminal.

## Integrate ROS 2 and Gazebo

### Install gazebo_ros_pkgs

1. Install the necessary packages for ROS 2 and Gazebo integration:

   bash
   sudo apt install ros-foxy-gazebo-ros-pkgs
   

### Test Your ROS 2 and Gazebo Integration

1. Load a demo robot using the following command:

   bash
   gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
2. In a new terminal window, send commands to make the robot move:

   bash
   ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1
   

3. Experiment with different commands and parameters to control the robot's movement.

   - Stop the robot:

     bash
     ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 0.0}}' -1
     

   - Change the robot's speed:

     bash
     ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 0.5}}' -1
     

4. Close all active programs by pressing CTRL + C in the terminals.

5. Relaunch Gazebo:

   bash
   gazebo
   

6. Insert your "Mobile Warehouse Robot" model into the environment and experiment with further commands.

## Build a Warehouse

### Create Model.config

1. Create a folder for the warehouse model:

   bash
   mkdir -p ~/.gazebo/models/small_warehouse
   

2. Create a model configuration file for the warehouse:

   bash
   gedit ~/.gazebo/models/small_warehouse/model.config
   

   Add information about the warehouse model, including name, version, author, and description.

3. Save the file and close it.

### Create Model.sdf

1. Create an SDF file for the warehouse model:

   bash
   gedit ~/.gazebo/models/small_warehouse/model.sdf
   

   Define the warehouse structure in the SDF file.

2. Save the file and close it.

### Test Your Warehouse

1. Run Gazebo:

   bash
   gazebo
   

2. In Gazebo, click the "Insert" tab and select "Small Warehouse." Place the warehouse model within the environment.

3. Experiment with building and customizing your warehouse using Gazebo's drag-and-drop interface.
