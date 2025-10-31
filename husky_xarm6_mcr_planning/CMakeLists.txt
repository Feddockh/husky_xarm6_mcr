cmake_minimum_required(VERSION 3.8)
project(husky_xarm6_mcr_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Find dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# Python dependencies
find_package(rclpy REQUIRED)

# Common dependencies (both C++ and Python)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ============================================================================
# C++ Includes
# ============================================================================
include_directories(include)

# ============================================================================
# C++ Libraries and Executables
# ============================================================================
# Build the MoveIt C++ interface library
add_library(${PROJECT_NAME}_lib
  src/moveit_cpp_interface.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)

# Example: C++ executable
# add_executable(cpp_planner_node src/cpp_planner_node.cpp)
# target_link_libraries(cpp_planner_node ${PROJECT_NAME}_lib)
# ament_target_dependencies(cpp_planner_node
#   rclcpp
#   moveit_ros_planning_interface
#   geometry_msgs
# )

# ============================================================================
# Python Setup
# ============================================================================
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/nbv_planner_node
  scripts/volumetric_map_node
  scripts/occupancy_map_node
  scripts/moveit_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Install
# ============================================================================
# Install C++ headers
install(DIRECTORY include/
  DESTINATION include
)

# Install C++ library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install C++ executables (uncomment when you have executables)
# install(TARGETS cpp_planner_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch, config, and rviz files
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install documentation
install(FILES
  README.md
  QUICK_START.md
  ROADMAP.md
  PLANNING_STRATEGIES.md
  DESTINATION share/${PROJECT_NAME}/
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# Export
# ============================================================================
# Export C++ dependencies for downstream packages
ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
)

ament_package()

