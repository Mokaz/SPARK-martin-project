import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('spark_softdrone'),
        'urdf',
        'drone.urdf.xacro'
    )

    robot_description_substitution = Command(['xacro ', xacro_file])

    # Then we hand that expanded URDF string to the robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_substitution}],
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
