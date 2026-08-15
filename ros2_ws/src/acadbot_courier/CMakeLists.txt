cmake_minimum_required(VERSION 3.8)
project(acadbot_courier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(acadbot_courier_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---- courier_server (RequestDelivery service + ExecuteDelivery action + Nav2 client) ----
add_executable(courier_server
  src/main.cpp
  src/courier_server.cpp
  src/location_table.cpp
  src/location_markers.cpp)
target_include_directories(courier_server PRIVATE include)
ament_target_dependencies(courier_server
  rclcpp rclcpp_action acadbot_courier_msgs nav2_msgs
  geometry_msgs tf2 tf2_geometry_msgs visualization_msgs)

install(TARGETS courier_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
