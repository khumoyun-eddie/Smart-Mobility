### Tasks

#### 1. Write the Subscriber Node with Statistics Enabled

Navigate to the `ros2_ws/src/cpp_pubsub/src` directory, which was created in the previous tutorial, and download the example subscriber code by running the following command:

```bash
wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp
```

Now, you will have a new file named `member_function_with_topic_statistics.cpp`. Open this file using your preferred text editor.

In this code, we have created a subscriber node that receives string messages from the "topic" and publishes statistics data for these messages. You can configure various options for topic statistics, including enabling/disabling statistics, setting the collection window, and defining the publish period.

#### 1.1 Examine the Code

The code allows you to manually enable topic statistics and configure parameters such as the collection window, publish period, and the topic for statistics publication. You can find these options in the code comments for reference.

#### 1.2 CMakeLists.txt

Open the `CMakeLists.txt` file and add an executable named `listener_with_topic_statistics` to run your node using `ros2 run`. This will ensure that your pub/sub system, with topic statistics enabled, is ready for use.

#### 2. Build and Run

To build and run the subscriber node with statistics enabled, follow the instructions provided in the pub/sub tutorial. Specifically, run the subscriber node using:

```bash
ros2 run cpp_pubsub listener_with_topic_statistics
```

And run the publisher node using:

```bash
ros2 run cpp_pubsub talker
```

The terminal will display messages being published by the publisher and received by the subscriber with statistics enabled.

#### 3. Observe Published Statistic Data

While the nodes are running, open a new terminal window and execute the following command:

```bash
ros2 topic list
```

You will see the `/statistics` topic (or the custom topic you may have defined) listed. This topic contains the statistics data published by the subscriber node.

To view the statistics data, use the following command:

```bash
ros2 topic echo /statistics
```

This will display statistics messages that include metrics like message age, message period, and other statistics based on the received messages.

### Summary

With this repository, you have created a ROS2 subscriber node with topic statistics enabled. This node publishes statistics data from the C++ publisher node, allowing you to monitor and analyze the performance and characteristics of the subscribed topic.