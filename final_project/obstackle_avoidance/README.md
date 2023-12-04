# Final project contribution: Lane Keeeping
**Author:** Khumoyun Rakhmonbediev 12194949

## Environment Setup:

- [`lane_keeping.world`] file was defined to setup the base environment with lane lines on ground plane.
- [`lane_keeping.sdf`] was defined to spawn the [modified TurtleBot3 Burger with a camera] in the [`lane_keeping.world`] environment.

&nbsp;
## Description:
The workspace for [`lane_keeping`] includes multiple ROS2 packages for accessing camera frames, vision processing, etc. that act as "helper packages" to the main [`lane_keeping`] package.

The ROS2 package [`lane_keeping`] for this assignment hosts the following [Python scripts]:
- [`lane_keeping.py`] makes the robot perform lane keeping operation. The robot is commanded to move with 0.22 m/s linear velocity. The incomming RGB camera frame (320x240 px) is cropped according to the region of interest (ROI) of 10 px height and entire width, converted to [HSV](https://en.wikipedia.org/wiki/HSL_and_HSV) color space, and masked into a binary image using upper and lower HSV thresholds for `yellow color`. The binary image is then used to calculate moments (weighted average of image pixel intensities) to detect cluster of pixels (i.e. blob), compute the centroid of this blob, and then this centroid is used to calculate error (deviation) of robot from lane center. A PID controller (with FIFO integral anti-windup mechanism) operates on this error to accordingly command the robot's angular velocity (rad/s) to perform lane keeping operation. The script is setup to wait 4 seconds for everything to initialize properly and then run for eternity to demonstrate dynamically variable lane keeping application.


The ROS2 package [`lane_keeping`] for this assignment hosts the following [launch files]:
- [`lane_keeping.launch.py`] launches [Gazebo simulator](https://gazebosim.org/home) with [`lane_keeping.sdf`], the [`lane_keeping.py`] node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the camera feed, laserscan and odometry estimates of the robot.

## Dependencies:
- [TurtleBot3 Burger Robot Hardware](https://www.robotis.us/turtlebot-3-burger-us/) with [TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)
- [TurtleBot3 Packages](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Simulations Packages](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Messages Package](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Dynamixel SDK Packages](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel) - Included with this repository

# Build:

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
    $ mv ~/Smart-Mobility/final_project/lane_keeping/ ~/ROS2_WS/src/
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
```bash
$ ros2 launch lane_keeping lane_keeping.launch.py
```

