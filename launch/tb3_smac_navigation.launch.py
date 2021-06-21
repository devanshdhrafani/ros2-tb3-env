#!/usr/bin/env python3

# Launch scipt for starting nav2 with smac planner
# Code modified from https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/bringup/launch/tb3_simulation_launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    # Get the launch directory
    tb3_env_dir = get_package_share_directory('tb3_env')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_bringup_dir, 'launch')


    world_file_name = 'turtlebot3_houses/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)

    map_file_name = 'house.yaml'
    map = os.path.join(get_package_share_directory('tb3_env'),'maps',map_file_name)

    rviz_config_file_name = 'smac_nav2_view.rviz'
    rviz_config_file = os.path.join(get_package_share_directory('tb3_env'),'rviz',rviz_config_file_name)

    params_file_name = 'smac_nav2_params.yaml'
    params_file = os.path.join(get_package_share_directory('tb3_env'),'params',params_file_name)

    # # Start Gazebo with house world file
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
    #     cwd=[launch_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     cmd=['gzclient'],
    #     cwd=[launch_dir], output='screen')

    # ros2 launch nav2_bringup tb3_simulation_launch.py <settings>
    # Where <settings> can used to replace any of the default options, for example:    
    # world:=<full/path/to/gazebo.world>
    # map:=<full/path/to/map.yaml>
    # rviz_config_file:=<full/path/to/rviz_config.rviz>
    # simulator:=<gzserver or gazebo>
    # bt_xml_file:=<full/path/to/bt_tree.xml>

    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'tb3_simulation_launch.py')
        ),
        launch_arguments={'world': world,
                        'map': map,
                        'rviz_config_file': rviz_config_file,
                        'params_file': params_file
                        }.items()
    )

    return LaunchDescription([
    # start_gazebo_server_cmd,
    # start_gazebo_client_cmd,
    # start_rviz_cmd
    nav2_bringup

    ])
