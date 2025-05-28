#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_state_publisher_launch_dir = os.path.join(
        get_package_share_directory('spark_softdrone'), 'launch', 'nodes'
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_state_publisher_launch_dir, 'robot_state_publisher.launch.py')
        )
    )

    t265_node = Node(
        package='spark_softdrone',
        executable='t265_node',
        name='t265_node',
        output='screen',
    )

    t265_to_map_tf_publisher_node = Node(
        package='spark_softdrone',
        executable='t265_to_map_tf_publisher',
        name='t265_to_map_tf_publisher',
        output='screen',
    )

    t265_odom_to_mavros_bridge_node = Node(
        package='spark_softdrone',
        executable='t265_odom_to_mavros_bridge',
        name='t265_odom_to_mavros_bridge_node',
        output='screen',
    )

    t265_pose_tf_broadcaster_node = Node(
        package='spark_softdrone',
        executable='t265_pose_tf_broadcaster',
        name='t265_pose_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_launch,
        t265_node,
        t265_to_map_tf_publisher_node,
        t265_odom_to_mavros_bridge_node,
        t265_pose_tf_broadcaster_node,
    ])
