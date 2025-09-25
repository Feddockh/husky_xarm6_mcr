cmake_minimum_required(VERSION 3.8)
project(husky_xarm6_mcr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# add_executable(plan_to_xyz src/plan_to_xyz.cpp)
# ament_target_dependencies(plan_to_xyz rclcpp moveit_ros_planning_interface tf2_geometry_msgs)

# add_executable(sanity_check src/sanity_check.cpp)
# ament_target_dependencies(sanity_check rclcpp moveit_ros_planning_interface)

# add_executable(plan_to_joint src/plan_to_joint.cpp)
# ament_target_dependencies(plan_to_joint rclcpp moveit_ros_planning_interface)

# add_executable(get_eef_pose src/get_eef_pose.cpp)
# ament_target_dependencies(get_eef_pose rclcpp moveit_ros_planning_interface tf2_geometry_msgs)

# add_executable(plan_linear_cart src/plan_linear_cart.cpp)
# ament_target_dependencies(plan_linear_cart rclcpp moveit_ros_planning_interface)

# add_executable(plan_square_cart src/plan_square_cart.cpp)
# ament_target_dependencies(plan_square_cart rclcpp moveit_ros_planning_interface)

# add_executable(plan_square_joint src/plan_square_joint.cpp)
# ament_target_dependencies(plan_square_joint rclcpp moveit_ros_planning_interface)

# install(TARGETS plan_to_xyz sanity_check plan_to_joint get_eef_pose plan_linear_cart plan_square_cart plan_square_joint
#         DESTINATION lib/${PROJECT_NAME})

ament_package()
