#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    xacro_file = "bruno.urdf.xacro"
    #xacro_file = "box_bot.xacro"
    package_description = "bruno"

    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", xacro_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()
    # Get Gazebo ROS interface package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the location for empty world
    world = os.path.join(
        get_package_share_directory('bruno'),
        'worlds',
        'empty_world.world'
    )

    # Launch Description to run Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Launch Description to run Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get the package directory 
    pkg_gazebo = get_package_share_directory('bruno')

   

    # Launch Decription to Spawn Robot Model 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch',
                         'spawn_robot_ros2.launch.py'),
        )
    )

     # RVIZ Configuration
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("bruno"), "rviz", "display_default.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    tf=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map',  '/dummy_link'  ],
    )

    # Launch Description 
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_world,
        robot_state_publisher,
        rviz_node,
        tf
        
    ])
