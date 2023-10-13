# To download the example talker code
wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp

# Build and run 
# to run subscriber
ros2 run cpp_pubsub listener_with_topic_statistics

# to run talker code
ros2 run cpp_pubsub talker

# to see currently active topics
ros2 topic list

# to view statistics data published
ros2 topic echo /statistics