# Hydra UAV - SPARKlab

## Overview
This repository contains the code for my work at SPARKlab during the IAP and spring semester of 2025 at MIT.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Iron
- Intel® RealSense™ T265 camera
- sudo apt-get install ros-iron-tf2 ros-iron-tf2-geometry-msgs
- sudo apt install ros-iron-xacro

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/SPARK-martin-project.git
cd SPARK-martin-project
```

### 2. Initialize Submodules
```bash
git submodule update --init --recursive
```

### 3. Install RealSense SDK
This project requires librealsense-2.53.1, which is the latest version compatible with the T265 camera.
For detailed installation instructions on how to install it on the Jetson, refer to this repo's wiki page on the subject.


### 5. Install MAVROS
```bash
# Install MAVROS packages
sudo apt-get install ros-iron-mavros ros-iron-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Usage
2. Start MAVROS:
```bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyTHS1:921600
```
3. Robot state publisher
```bash
ros2 launch spark_softdrone robot_state_publisher.launch.py
```
4. Start T265 node
```bash
ros2 run spark_softdrone t265_node
```
5. Start T265 to map publisher
```bash
ros2 run spark_softdrone t265_to_map_tf_publisher
```
6. Start T265 odometry to MAVROS bridge node:
```bash
ros2 run spark_softdrone t265_odom_to_mavros_bridge
```
7. px4_local_position_tf_broadcaster
```bash
ros2 run spark_softdrone px4_local_position_tf_broadcaster 
```

8. Start the RealSense camera node:
```bash
ros2 launch realsense2_camera rs_launch.py base_frame_id:=rgbd_link
```

Or launch everything at once: (OLD)
```bash
ros2 launch spark_softdrone mavros_t265_vio.launch.py
```

## Example code
1. Start and arm drone in offboard mode:
```bash
ros2 run spark_softdrone offboard_control_srv
```
2. Start GUI for publishing setpoints (on GCS computer):
```bash
ros2 run spark_softdrone setpoint_gui.py
```

## Connect to drone through telemetry radio
On Ground Computer connect telemetry radio and run:
```bash
screen /dev/ttyUSB0 57600
```

## Troubleshooting
- If MAVROS complains about connection to Pixhawk though serial, make sure Jetson has permission to use serial device:
```bash
sudo chmod 666 /dev/ttyTHS1
```
