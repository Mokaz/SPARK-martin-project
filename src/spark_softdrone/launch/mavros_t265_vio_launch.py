#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch'
    )
    mavros_launch_dir = os.path.join(
        get_package_share_directory('mavros'), 'launch'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        )
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavros_launch_dir, 'px4.launch')
        ),
        launch_arguments={
            'fcu_url': 'serial:///dev/ttyTHS1:921600'
        }.items()
    )

    t265_to_mavros_node = Node(
        package='spark_softdrone',
        executable='t265_to_mavros',
        name='t265_to_mavros_node',
        output='screen',
    )

    return LaunchDescription([
        realsense_launch,
        mavros_launch,
        t265_to_mavros_node
    ])
