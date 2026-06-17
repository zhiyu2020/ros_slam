cmake_minimum_required(VERSION 3.8)
project(slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(keyboard_vel_cmd src/keyboard_vel_cmd.cpp)
ament_target_dependencies(keyboard_vel_cmd
  rclcpp
  geometry_msgs
)

add_executable(sim_observer src/sim_observer.cpp)
ament_target_dependencies(sim_observer
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(gazebo_model_odom src/gazebo_model_odom.cpp)
ament_target_dependencies(gazebo_model_odom
  gazebo_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(
  DIRECTORY
    config
    launch
    meshes
    models
    rviz
    urdf
    worlds
    xacro
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    gazebo_model_odom
    keyboard_vel_cmd
    sim_observer
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
