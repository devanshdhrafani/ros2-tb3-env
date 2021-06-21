#!/usr/bin/env python3

"""
-Script used to spawn tb3 in gazebo house environment and start SLAM
-To move robot, use teleop_key or any other method of choice
-Save map using
    ros2 run nav2_map_server map_saver_cli -f ~/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
    
    tb3_gazebo_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    tb3_gazebo_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_launch_file_dir, 'turtlebot3_house.launch.py')
        )
    )

    # ros2 launch slam_toolbox online_async_launch.py

    slam_toolbox_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_file_dir, 'online_async_launch.py')
        )
    )

    # ros2 run rviz2 rviz2

    tb3_env_pkg_dir = get_package_share_directory('tb3_env')
    rviz_config_file = os.path.join(tb3_env_pkg_dir, 'rviz', 'slam_view.rviz')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([
        tb3_gazebo_house,
        start_slam,
        start_rviz_cmd
    ])
