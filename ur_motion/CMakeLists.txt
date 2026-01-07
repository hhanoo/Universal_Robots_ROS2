cmake_minimum_required(VERSION 3.8)
project(ur_motion)

# =========================================================
# Compiler options
# =========================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# ROS2 build system
# =========================================================
find_package(ament_cmake REQUIRED)

# =========================================================
# ROS2 / MoveIt dependencies
# =========================================================
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# =========================================================
# Action generation
# =========================================================
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJ.action"
  "action/MoveL.action"
)

ament_export_dependencies(rosidl_default_runtime)

# =========================================================
# Library: ur_motion_plugin
#  - Backend들을 외부 패키지(opcua_server)에서 직접 사용하기 위한 라이브러리
# =========================================================
add_library(ur_motion_plugin SHARED
  src/moveit_backend.cpp
  src/transform_utils.cpp
)

target_include_directories(ur_motion_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ur_motion_plugin
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# =========================================================
# Executable: ur_action_server
#  - Action server는 backend 라이브러리를 링크해서 사용
# =========================================================
add_executable(ur_action_server
  src/ur_action_server.cpp
)

target_link_libraries(ur_action_server
  ur_motion_plugin
)

ament_target_dependencies(ur_action_server
  rclcpp
  rclcpp_action
)

# Action typesupport linking
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ur_action_server
  "${cpp_typesupport_target}"
)

# =========================================================
# Install
# =========================================================
# Install executable and library targets
install(
  TARGETS
    ur_motion_plugin
    ur_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install public header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install action definition files
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================
# Export dependencies (Action interface only)
# =========================================================
ament_export_dependencies(
  rosidl_default_runtime  # Action interface dependencies (automatically exports headers)
)

# =========================================================
# Testing
# =========================================================
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

# =========================================================
# Package declaration
# =========================================================
ament_package()
