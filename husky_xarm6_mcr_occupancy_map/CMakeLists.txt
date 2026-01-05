cmake_minimum_required(VERSION 3.22)
project(husky_xarm6_mcr_occupancy_map)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoxelGrid.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
)

# ============================================================================
# Core Library: Thread-safe Octomap wrapper
# ============================================================================
add_library(${PROJECT_NAME}_core
  src/occupancy_map_tree.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
)

# ============================================================================
# Monitor Library: Map management and updating
# ============================================================================
add_library(${PROJECT_NAME}_monitor
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  src/pointcloud_updater.cpp
)
target_include_directories(${PROJECT_NAME}_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_monitor
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================================
# Visualization Library: RViz markers for debugging
# ============================================================================
add_library(${PROJECT_NAME}_visualization
  src/occupancy_map_visualizer.cpp
)
target_include_directories(${PROJECT_NAME}_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_visualization
  ${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_visualization
  rclcpp
  visualization_msgs
  std_msgs
)

# ============================================================================
# Voxel Grid Publisher Library (for NBV planner)
# ============================================================================
add_library(${PROJECT_NAME}_voxel_grid
  src/voxel_grid_publisher.cpp
)
target_include_directories(${PROJECT_NAME}_voxel_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_voxel_grid
  ${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_voxel_grid
  rclcpp
  std_msgs
)

# Link against our own generated messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_voxel_grid "${cpp_typesupport_target}")

# ============================================================================
# Executables
# ============================================================================

# Standalone occupancy map server (publishes octomap_msgs)
add_executable(occupancy_map_server
  src/occupancy_map_server.cpp
)
target_link_libraries(occupancy_map_server
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_visualization
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(occupancy_map_server
  rclcpp
  sensor_msgs
  tf2_ros
  octomap_msgs
  moveit_msgs
  visualization_msgs
)

# MoveIt integration node
add_executable(moveit_octomap_server
  src/moveit_octomap_server.cpp
)
target_link_libraries(moveit_octomap_server
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_visualization
  ${PROJECT_NAME}_voxel_grid
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(moveit_octomap_server
  rclcpp
  sensor_msgs
  tf2_ros
  octomap_msgs
  moveit_msgs
  visualization_msgs
)

# Link against our own generated messages
target_link_libraries(moveit_octomap_server "${cpp_typesupport_target}")

# ============================================================================
# Install
# ============================================================================

# Install libraries
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_visualization
  ${PROJECT_NAME}_voxel_grid
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
  occupancy_map_server
  moveit_octomap_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# ============================================================================
# Export
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  octomap
  octomap_msgs
  moveit_msgs
)
ament_export_include_directories(include)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
