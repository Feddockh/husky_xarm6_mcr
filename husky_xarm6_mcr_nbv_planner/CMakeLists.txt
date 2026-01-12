cmake_minimum_required(VERSION 3.8)
project(husky_xarm6_mcr_nbv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Define the shared library
add_library(moveit_interface SHARED
  src/moveit_interface.cpp
)
target_include_directories(moveit_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(moveit_interface
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# Define the node executable
add_executable(moveit_interface_demo src/moveit_interface_demo.cpp)
target_link_libraries(moveit_interface_demo moveit_interface)
ament_target_dependencies(moveit_interface_demo
  rclcpp
)

# Install rules
install(TARGETS moveit_interface moveit_interface_demo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
