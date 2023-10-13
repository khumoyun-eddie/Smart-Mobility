# we need rosbag2 packages installed
sudo apt install ros-humble-rosbag2

# create ros2 packages
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs
