cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficLightState.msg"
  DEPENDENCIES std_msgs
)

add_executable(sensor_tf_broadcaster_node src/sensor_tf_broadcaster_node.cpp)
ament_target_dependencies(sensor_tf_broadcaster_node rclcpp geometry_msgs tf2_ros)

add_executable(depth_to_pointcloud_node src/depth_to_pointcloud_node.cpp)
ament_target_dependencies(depth_to_pointcloud_node rclcpp sensor_msgs message_filters)

add_executable(traffic_light_detector_node src/traffic_light_detector_node.cpp)
ament_target_dependencies(traffic_light_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(traffic_light_detector_node "${cpp_typesupport_target}")

install(TARGETS
  sensor_tf_broadcaster_node
  depth_to_pointcloud_node
  traffic_light_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
