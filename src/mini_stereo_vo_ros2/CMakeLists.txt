cmake_minimum_required(VERSION 3.20)
project(mini_stereo_vo_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../mini_stereo_vo
  ${CMAKE_CURRENT_BINARY_DIR}/mini_stereo_vo
)

add_executable(stereo_vo_node src/stereo_vo_node.cpp)

target_link_libraries(stereo_vo_node svo_core)

ament_target_dependencies(stereo_vo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  image_transport
  cv_bridge
  message_filters
)

install(TARGETS stereo_vo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/bag2tum.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/run_waypoints.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
