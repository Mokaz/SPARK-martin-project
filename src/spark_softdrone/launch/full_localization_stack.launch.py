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
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch'
    )
    mavros_launch_dir = os.path.join(
        get_package_share_directory('mavros'), 'launch'
    )
    mavros_px4_config_dir = os.path.join(
        get_package_share_directory('spark_softdrone'), 'config'
    )

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(mavros_launch_dir, 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': 'serial:///dev/ttyTHS1:921600',
            'config_yaml': os.path.join(mavros_px4_config_dir, 'mavros_px4_config_spark.yaml'),
        }.items()
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

    px4_local_position_tf_broadcaster_node = Node(
        package='spark_softdrone',
        executable='px4_local_position_tf_broadcaster',
        name='px4_local_position_tf_broadcaster',
        output='screen',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ), 
        launch_arguments={
            'base_frame_id': 'rgbd_link',
            'depth_module.profile': '1280x720x30',
            'rgb_camera.profile': '1280x720x30',
            'device_type': 'D455',
        }.items()
    )

    return LaunchDescription([
        mavros_launch,
        robot_state_publisher_launch,
        t265_node,
        t265_to_map_tf_publisher_node,
        t265_odom_to_mavros_bridge_node,
        px4_local_position_tf_broadcaster_node,
        realsense_launch
    ])
