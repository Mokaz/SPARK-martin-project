cmake_minimum_required(VERSION 3.5)
project(spark_softdrone)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
# add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
# ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
# target_include_directories(frame_transforms PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
# 	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# offboard_control_srv
add_executable(offboard_control_srv src/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# visualizer.cpp
add_executable(visualizer src/visualizer.cpp)
ament_target_dependencies(visualizer rclcpp px4_msgs nav_msgs visualization_msgs)
install(TARGETS visualizer DESTINATION lib/${PROJECT_NAME})

# t265_odom_to_mavros_bridge.cpp
add_executable(t265_odom_to_mavros_bridge src/t265_odom_to_mavros_bridge.cpp)
ament_target_dependencies(t265_odom_to_mavros_bridge rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS t265_odom_to_mavros_bridge DESTINATION lib/${PROJECT_NAME})

# px4_local_position_tf_broadcaster.cpp
add_executable(px4_local_position_tf_broadcaster src/px4_local_position_tf_broadcaster.cpp)
ament_target_dependencies(px4_local_position_tf_broadcaster rclcpp px4_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS px4_local_position_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

# t265_to_map_tf_publisher.cpp
add_executable(t265_to_map_tf_publisher src/t265_to_map_tf_publisher.cpp)
ament_target_dependencies(t265_to_map_tf_publisher rclcpp tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS t265_to_map_tf_publisher DESTINATION lib/${PROJECT_NAME})

# t265_pose_tf_broadcaster.cpp
add_executable(t265_pose_tf_broadcaster src/t265_pose_tf_broadcaster.cpp)
ament_target_dependencies(t265_pose_tf_broadcaster rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros)
install(TARGETS t265_pose_tf_broadcaster DESTINATION lib/${PROJECT_NAME})

# t265_node.cpp
add_executable(t265_node src/t265_node.cpp)
ament_target_dependencies(t265_node rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(t265_node realsense2)   # librealsense
install(TARGETS t265_node DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs)

# ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
# ament_export_libraries(frame_transforms)

# Install header files
# install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# install(TARGETS frame_transforms
# 	EXPORT export_frame_transforms
# 	ARCHIVE DESTINATION lib
# 	LIBRARY DESTINATION lib
# 	RUNTIME DESTINATION bin
# 	INCLUDES DESTINATION include
# )

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install urdf files.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)

# Install meshes
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)

# Install config files.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install tests
# install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/visualizer.py
  src/setpoint_gui.py
  scripts/servo_pivot_joint_state_publisher.py
  scripts/pose_serial_sender.py
  scripts/pose_serial_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
