cmake_minimum_required(VERSION 3.8)
project(slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(keyboard_vel_cmd src/keyboard_vel_cmd.cpp)
ament_target_dependencies(keyboard_vel_cmd
  rclcpp
  geometry_msgs
)

install(
  DIRECTORY
    config
    launch
    meshes
    models
    rviz
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    keyboard_vel_cmd
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
