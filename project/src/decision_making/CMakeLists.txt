cmake_minimum_required(VERSION 3.8)
project(decision_making)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(planning REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HazardStatus.msg"
  DEPENDENCIES std_msgs
)

add_executable(forward_obstacle_monitor_node src/forward_obstacle_monitor_node.cpp)
ament_target_dependencies(forward_obstacle_monitor_node rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros
  tf2_geometry_msgs planning)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(forward_obstacle_monitor_node "${cpp_typesupport_target}")

add_executable(decision_state_machine_node src/decision_state_machine_node.cpp)
ament_target_dependencies(decision_state_machine_node rclcpp std_msgs geometry_msgs planning)
target_link_libraries(decision_state_machine_node "${cpp_typesupport_target}")

install(TARGETS
  forward_obstacle_monitor_node
  decision_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
