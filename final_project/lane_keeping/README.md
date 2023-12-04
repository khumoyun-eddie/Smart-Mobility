# Final project contribution: TurtleBot3 Emergency Braking, Wall Following and Obstacle Avoidance
**Author:** Khumoyun Rakhmonberdiev 12194949

## Description:
The ROS2 package includes:
- [`emergency_braking.py`]makes the robot stop if obstacles are detected within a threshold distance. The robot is commanded to move with 1 m/s linear velocity unless any obstacle is detected. In case any obstacle is detected right in front of the robot within a threshold distance of 1 m, the robot is commanded to perform emergency braking. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable emergency braking application.
- [`wall_following.py`] makes the robot follow walls by maintaining equal distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and relative distance from walls respectively for motion control. A simple finite state machine is implemented to account for `inf` scan measurements beyond LIDAR maximum range. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable wall following application.
- [`obstacle_avoidance.py`] makes the robot avoid obstacles by maintaining a safe distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and minimum distance to collision from left and right sectors spanning 30° each respectively for motion control. A finite state machine is implemented to account for turning left, turning right, going straight and cautiously rotating on the spot in case of too many obstacle in proximity. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable obstacle avoidance application.
- [`collision_avoidance.py`] makes the robot avoid collision with obstacles by maintaining a safe distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and minimum distance to collision from left and right sectors spanning 30° each respectively for motion control. A finite state machine is implemented to account for turning left, turning right, going straight and cautiously rotating on the spot in case of too many obstacle in proximity. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable obstacle avoidance application. It is worth mentioning that with TurtleBot3, it is either possible to subscribe to sensor data or publish actuator commands (but NOT both) when interfaced with ROS-2 over a single [`Quality of Service (QoS)`](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html) profile of the underlying [Data Distribution Service (DDS) or Real-Time Publish Subscribe (RTPS)](https://design.ros2.org/articles/ros_on_dds.html) implementation (also, [ROS-2 supports multiple DDS/RTPS implementations](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html)). Hence, we have created different QoS profiles for subscribing to sensor data (using `best effort` communication and small queue depth) and publishing actuator commands (using `reliable` communication and relatively large queue depth).

The ROS2 package [`obstackle_avoidance`] has launch files:
- [`emergency_braking.launch.py`] launches [Gazebo simulator](https://gazebosim.org/home), the [`emergency_braking.py`] node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`wall_following.launch.py`] launches [Gazebo simulator](https://gazebosim.org/home), the [`wall_following.py`] node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`obstacle_avoidance.launch.py`] launches [Gazebo simulator](https://gazebosim.org/home), the [`obstacle_avoidance.py`] node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`collision_avoidance.launch.py`] the [`collision_avoidance.py`] node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.

## Dependencies:
- [TurtleBot3 Burger Robot Hardware](https://www.robotis.us/turtlebot-3-burger-us/) with [TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)
- [TurtleBot3 Packages](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Simulations Packages](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Messages Package](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Dynamixel SDK Packages](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel) - Included with this repository

## Build:

1. Make a directory `ROS2_WS` to act as your ROS2 workspace.
    ```bash
    $ mkdir -p ~/ROS2_WS/src/
    ```
2. Clone this repository:
    ```bash
    $ git clone https://github.com/khumoyun-eddie/Smart-Mobility.git
    ```
3. Move `final_project` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Smart-Mobility/final_project/obstackle_avoidance/ ~/ROS2_WS/src/
    ```
4. [Optional] Remove the unnecessary files.
    ```bash
    $ sudo rm -r Smart-Mobility
    ```
5. Build the packages.
    ```bash
    $ cd ~/ROS2_WS
    $ colcon build
    ```
6. Source the `setup.bash` file of your `ROS2_WS`.
    ```bash
    $ echo "source ~/ROS2_WS/install/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

## Execute:
### Simulation:
1. Emergency Braking:
    ```bash
    $ ros2 launch obstackle_avoidance emergency_braking.launch.py
    ```
2. Wall Following:
    ```bash
    $ ros2 launch obstackle_avoidance wall_following.launch.py
    ```
3. Obstacle Avoidance:
    ```bash
    $ ros2 launch obstackle_avoidance obstacle_avoidance.launch.py
    ```
### Real World:
1. Connect to the TurtleBot3 SBC via Secure Shell Protocol (SSH):
    ```bash
    user@computer:~$ sudo ssh <username>@<ip.address.of.turtlebot3>
    user@computer:~$ sudo ssh ubuntu@192.168.43.48
    ```
2. Bringup TurtleBot3:
    ```bash
    ubuntu@ubuntu:~$ ros2 launch turtlebot3_bringup robot.launch.py
    ```   
3. Collision Avoidance:
    ```bash
    user@computer:~$ ros2 launch obstackle_avoidance collision_avoidance.launch.py
    ```
