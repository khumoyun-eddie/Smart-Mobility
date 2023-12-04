# Author: Khumoyun Rakhmonberdiev 12194949

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
    pkg_share = get_package_share_directory('obstackle_avoidance')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    turtlebot3_gazebo_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_share, 'worlds', 'wall_following.sdf')],
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
            package='obstackle_avoidance',
            executable='wall_following',
            name='wall_following_node',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='odometry_rviz',
            arguments=['-d', [FindPackageShare("obstackle_avoidance"), '/rviz', '/obstackle_avoidance.rviz',]]
        ),
    ])