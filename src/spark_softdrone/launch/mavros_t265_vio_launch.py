from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the path to the realsense2_camera package
    realsense_pkg_dir = FindPackageShare('realsense2_camera').find('realsense2_camera')
    realsense_launch_path = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    # Get the path to the mavros package
    mavros_pkg_dir = FindPackageShare('mavros').find('mavros')
    mavros_launch_path = os.path.join(mavros_pkg_dir, 'launch', 'px4.launch')

    # Create the launch description
    ld = LaunchDescription()

    # Add the RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path)
    )
    ld.add_action(realsense_launch)

    # Add the MAVROS launch with FCU URL configuration
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={'fcu_url': 'serial:///dev/ttyTHS1:921600'}.items()
    )
    ld.add_action(mavros_launch)

    # Add the t265_to_mavros node
    t265_to_mavros_node = Node(
        package='spark_softdrone',
        executable='t265_to_mavros',
        name='t265_to_mavros'
    )
    ld.add_action(t265_to_mavros_node)

    return ld