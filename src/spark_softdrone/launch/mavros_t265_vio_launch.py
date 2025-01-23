#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch'
    )
    mavros_launch_dir = os.path.join(
        get_package_share_directory('mavros'), 'launch'
    )
    mavros_px4_config_dir = os.path.join(
        get_package_share_directory('spark_softdrone'), 'launch', 'px4_config_spark.yaml'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        )
    )

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(mavros_launch_dir, 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': 'serial:///dev/ttyTHS1:921600',
            'config_yaml': mavros_px4_config_dir
        }.items()
    )

    t265_to_mavros_node = Node(
        package='spark_softdrone',
        executable='t265_to_mavros',
        name='t265_to_mavros_node',
        output='screen',
    )

    return LaunchDescription([
        mavros_launch,
        realsense_launch,
        t265_to_mavros_node
    ])
