cmake_minimum_required(VERSION 3.8)
project(ur_robot_client)

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
# ROS2 dependencies
# =========================================================
# * ROS 2 core client libraries
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# * UR Motion action interface
find_package(ur_motion REQUIRED)

# * UR-specific messages and services
find_package(ur_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

# * Common message types (기본 메시지 타입)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# * TF2 for TCP pose tracking
find_package(tf2_ros REQUIRED)

# =========================================================
# Include directories
# =========================================================
include_directories(include)

# =========================================================
# Library: ur_control_client_lib
#  - Reusable client library for UR robot control
#  - 다른 패키지에서 재사용 가능한 UR 로봇 제어 클라이언트 라이브러리
# =========================================================
add_library(${PROJECT_NAME}_lib SHARED
  src/ur_robot_client.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  ur_motion
  ur_msgs
  ur_dashboard_msgs
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2_ros
)

# =========================================================
# Example executables (예제 실행 파일)
# =========================================================
# Example 1: MoveJ (joint space motion)
# 예제 1: MoveJ (관절 공간 모션)
add_executable(example_movej src/example_movej.cpp)
target_link_libraries(example_movej ${PROJECT_NAME}_lib)
ament_target_dependencies(example_movej
  rclcpp
)

# Example 2: I/O and Speed control
# 예제 2: I/O 및 속도 제어
add_executable(example_io_speed src/example_io_speed.cpp)
target_link_libraries(example_io_speed ${PROJECT_NAME}_lib)
ament_target_dependencies(example_io_speed
  rclcpp
)

# Example 3: Complete Pick & Place scenario
# 예제 3: 완전한 Pick & Place 시나리오
add_executable(example_complete src/example_complete.cpp)
target_link_libraries(example_complete ${PROJECT_NAME}_lib)
ament_target_dependencies(example_complete
  rclcpp
)

# Example 4: Read-only state monitoring
# 예제 4: 읽기 전용 상태 모니터링
add_executable(example_state src/example_state.cpp)
target_link_libraries(example_state ${PROJECT_NAME}_lib)
ament_target_dependencies(example_state
  rclcpp
)

# =========================================================
# Install
# =========================================================
# Install executable and library targets
install(
  TARGETS
    ${PROJECT_NAME}_lib
    example_movej
    example_io_speed
    example_complete
    example_state
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install public header files
install(
  DIRECTORY include/
  DESTINATION include
)

# =========================================================
# Export information (외부 패키지에서 사용 가능하도록 정보 내보내기)
# =========================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  ur_motion
  ur_msgs
  ur_dashboard_msgs
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2_ros
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
# Package declaration (패키지 선언)
# =========================================================
ament_package()
