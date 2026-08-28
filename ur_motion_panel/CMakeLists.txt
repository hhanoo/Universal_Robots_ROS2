cmake_minimum_required(VERSION 3.8)
project(ur_motion_panel)

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

# * UR robot control client library
find_package(ur_robot_client REQUIRED)

# =========================================================
# External dependencies
# =========================================================
# * Qt6 (Widgets: GUI, Concurrent: motion worker threads)
#   Optional: hosts without Qt6 (headless consumers) get an empty package
find_package(Qt6 QUIET COMPONENTS Widgets Concurrent)
if(NOT Qt6_FOUND)
  message(STATUS "Qt6 not found; skipping ${PROJECT_NAME}")
  ament_package()
  return()
endif()

# * Eigen3 for TCP rotation math
find_package(Eigen3 REQUIRED)

# =========================================================
# Qt automatic code generation (moc/uic)
#  - AUTOUIC picks up src/mainwindow.ui via #include "ui_mainwindow.h"
# =========================================================
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# =========================================================
# Executable: ur_motion_panel
# =========================================================
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  src/setting_config.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt6::Widgets
  Qt6::Concurrent
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ur_robot_client
)

# =========================================================
# Install
# =========================================================
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
