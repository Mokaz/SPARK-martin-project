# Hydra UAV - SPARKlab

## Overview
This repository contains the code for running Hydra on the drone.

## Prerequisites
- Intel® RealSense™ T265 and D455 cameras 
- Ubuntu 22.04 LTS
- ROS2 Iron
- MAVROS for ROS2 Iron
- librealsense-2.53.1: the latest version compatible with the T265 camera. For detailed installation instructions on how to install it on the Jetson, refer to this repo's [wiki page](https://github.com/Mokaz/SPARK-martin-project/wiki/Installing-Prerequisites) on the subject.
- realsense-ros build 4.54.1: Note that changes to this package will be needed; please refer to the wiki page mentioned above for instructions. 

## Quickstart

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/SPARK-martin-project.git
cd SPARK-martin-project
```

### 2. Initialize Submodules
```bash
git submodule update --init --recursive
```

### 3. Build using colocon from your workspace
```bash
colcon build
```

## Usage
1. Start MAVROS:
```bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyTHS1:921600
```
2. Robot state publisher
```bash
ros2 launch spark_softdrone robot_state_publisher.launch.py
```
3. Start T265 node
```bash
ros2 run spark_softdrone t265_node
```
4. Start T265 to map publisher
```bash
ros2 run spark_softdrone t265_to_map_tf_publisher
```
5. Start T265 odometry to MAVROS bridge node:
```bash
ros2 run spark_softdrone t265_odom_to_mavros_bridge
```
6. px4_local_position_tf_broadcaster
```bash
ros2 run spark_softdrone px4_local_position_tf_broadcaster 
```

7. Start the RealSense camera node for D455:
```bash
ros2 launch realsense2_camera rs_launch.py \
    base_frame_id:=rgbd_link \
    depth_module.profile:=1280x720x30 \
    rgb_camera.profile:=1280x720x30 \
    device_type:=D455
```

Or launch everything at once: (Unstable)
```bash
ros2 launch spark_softdrone full_localization_stack.launch.py
```

Or launch nodes 2 through 5 + [t265_pose_tf_broadcaster](https://github.com/Mokaz/SPARK-martin-project/blob/main/src/spark_softdrone/src/t265_pose_tf_broadcaster.cpp) (more stable, no MAVROS):
```bash
ros2 launch spark_softdrone pure_t265_localization.launch.py
```

## Run Hydra
If you haven't already, make sure to make a separate workspace for Hydra and install it there. Follow the [guide](https://github.com/Mokaz/SPARK-martin-project/wiki/Running-Hydra) on the wiki.
### Source and run Hydra
```bash
ros2 launch hydra_ros UAV.launch.yaml
```

### Run Semantic segmentation
```bash
ros2 launch semantic_inference_ros closed_set.yaml model_file:=ade20k-efficientvit_seg_l2.onnx
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

## Visualization
Source the installation and run rviz2
```bash
ros2 run rviz2 rviz2 --ros-args -d /src/spark_softdrone/rviz/localization.rviz
```

## Connect to drone through telemetry radio
Connect the telemetry radio to the Jetson by USB on the drone. See details [wiki page on Baseboard setup](https://github.com/Mokaz/SPARK-martin-project/wiki/Baseboard-setup#jetson-configured-system-services) for info on how this was set up.

On Ground Computer connect the telemetry radio and run:
```bash
screen /dev/ttyUSB0 57600
```

## Troubleshooting
- If MAVROS complains about connection to Pixhawk though serial, make sure Jetson has permission to use serial device:
```bash
sudo chmod 666 /dev/ttyTHS1
```
- If T265 node does not publish any topics, simply unplug and replug the T265 camera. 
