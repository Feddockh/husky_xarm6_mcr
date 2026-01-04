cmake_minimum_required(VERSION 3.8)
project(husky_xarm6_mcr_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build map_frame_publisher executable
# add_executable(map_frame_publisher src/map_frame_publisher.cpp)
# ament_target_dependencies(map_frame_publisher
#   rclcpp
#   tf2_ros
#   geometry_msgs
# )

# # Install executables
# install(TARGETS
#   map_frame_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
