import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('lane_keeping')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    turtlebot3_gazebo_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_share, 'worlds', 'lane_keeping.sdf')],
            description='Simulation Description Format (SDFormat/SDF) for Describing Robot and Environment',
        ),
        gazebo,
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_launch, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='lane_keeping',
            executable='lane_keeping',
            name='lane_keeping_node',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='odometry_rviz',
            arguments=['-d', [FindPackageShare("lane_keeping"), '/rviz', '/lane_keeping.rviz',]]
        ),
    ])
