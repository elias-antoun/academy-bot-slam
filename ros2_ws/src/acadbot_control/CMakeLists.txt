cmake_minimum_required(VERSION 3.8)
project(acadbot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- square_driver (Session 1 drift demo) ----
add_executable(square_driver src/square_driver.cpp)
ament_target_dependencies(square_driver rclcpp geometry_msgs)

# ---- patrol_commander (Session 4 Nav2 action client) ----
add_executable(patrol_commander src/patrol_commander.cpp)
ament_target_dependencies(patrol_commander
  rclcpp rclcpp_action geometry_msgs nav2_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  square_driver
  patrol_commander
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
