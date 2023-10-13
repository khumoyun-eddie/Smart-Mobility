# First setup discovery server
fastdds discovery --server-id 0

# in the new terminal set the env
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# launch listener node
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

# launch talker node in the new terminal
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

# finally, to check everything is running correctly
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker