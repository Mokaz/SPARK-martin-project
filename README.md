# SPARK-martin-project

## Overview
This repository contains the code for my work at SPARKlab during the IAP and spring semester of 2025 at MIT.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Intel® RealSense™ T265 camera

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
For detailed installation instructions, refer to the [official RealSense SDK documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md).


### 5. Install MAVROS
```bash
# Install MAVROS packages
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

## Usage
1. Start the RealSense camera node:
```bash
ros2 launch realsense2_camera rs_launch.py
```

2. Start MAVROS:
```bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyTHS1:921600
```
3. Start T265 to MAVROS node:
```bash
ros2 run spark_softdrone t265_to_mavros
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

## Troubleshooting
- If MAVROS complains about connection to Pixhawk though serial, make sure Jetson has permission to use serial device:
```bash
sudo chmod 666 /dev/ttyTHS1
```
