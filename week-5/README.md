# ROS2 Dependency Management and Action Server/Client

This README provides instructions for managing ROS2 dependencies using `rosdep` and creating an action server and client in C++. Additionally, it covers runtime and compile-time composition using ROS services and dlopen. These steps will help you get started with ROS2 development.

## Table of Contents
1. [Managing ROS2 Dependencies with `rosdep`](#managing-ros2-dependencies)
2. [Creating an Action](#creating-an-action)
3. [Writing an Action Server and Client (C++)](#writing-an-action-server-and-client-c++)
4. [Adding Visibility Control](#adding-visibility-control)
5. [Composing Multiple Nodes in a Single Process](#composing-multiple-nodes)
6. [Run-Time Composition using ROS Services](#run-time-composition-ros-services)
7. [Compile-Time Composition using ROS Services](#compile-time-composition-ros-services)
8. [Run-Time Composition using dlopen](#run-time-composition-dlopen)

### 1. Managing ROS2 Dependencies with `rosdep`<a name="managing-ros2-dependencies"></a>

To manage ROS2 dependencies, follow these steps:

```bash
# Install rosdep
sudo apt-get install python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Install dependencies for your workspace
rosdep install --from-paths src -y --ignore-src
```

### 2. Creating an Action<a name="creating-an-action"></a>

To create a ROS2 action, perform the following steps:

```bash
# Set up a workspace and create a package named action_tutorials_interfaces
mkdir -p ros2_ws/src  # You can reuse an existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

# Create an action directory in the package
cd action_tutorials_interfaces
mkdir action

# Modify CMakeLists.txt and package.xml as per the provided code
# (Refer to the original script for the code to add)
# ...

# Build the package containing the action definition
cd ~/ros2_ws
colcon build
```

### 3. Writing an Action Server and Client (C++)<a name="writing-an-action-server-and-client-c++"></a>

To write an action server and client in C++, follow these steps:

```bash
# Create a new package for the C++ action server
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

# Build the workspace
cd ~/ros2_ws
colcon build
```

For additional details and visibility control, refer to [this link](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#adding-in-visibility-control).

### 4. Adding Visibility Control<a name="adding-visibility-control"></a>

For visibility control, please refer to the tutorial at [this link](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#adding-in-visibility-control).

### 5. Composing Multiple Nodes in a Single Process<a name="composing-multiple-nodes"></a>

To compose multiple nodes in a single process, follow these steps:

```bash
# See available components in the workspace
ros2 component types

# Run the component container
ros2 run rclcpp_components component_container

# In another terminal, load the Talker component
ros2 component load /ComponentManager composition composition::Talker
```

### 6. Run-Time Composition using ROS Services<a name="run-time-composition-ros-services"></a>

For run-time composition using ROS services, follow these steps:

```bash
# Run the component container
ros2 run rclcpp_components component_container

# In another terminal, load the Server and Client components
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```

### 7. Compile-Time Composition using ROS Services<a name="compile-time-composition-ros-services"></a>

For compile-time composition using ROS services, execute the following command:

```bash
ros2 run composition manual_composition
```

### 8. Run-Time Composition using dlopen<a name="run-time-composition-dlopen"></a>

To perform run-time composition using `dlopen`, use the following commands:

```bash
# Run the dlopen composition
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```

This README provides an overview of managing ROS2 dependencies and creating action servers/clients and composing nodes. For detailed instructions and additional resources, please refer to the provided links and the ROS2 documentation.