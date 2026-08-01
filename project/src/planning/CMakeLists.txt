cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(waypoint_recorder_node src/waypoint_recorder_node.cpp)
ament_target_dependencies(waypoint_recorder_node rclcpp geometry_msgs)
target_link_libraries(waypoint_recorder_node yaml-cpp::yaml-cpp)

add_executable(goal_selector_node src/goal_selector_node.cpp)
ament_target_dependencies(goal_selector_node rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(goal_selector_node yaml-cpp::yaml-cpp)

add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)
ament_target_dependencies(trajectory_planner_node rclcpp std_msgs geometry_msgs nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_planner_node "${cpp_typesupport_target}")

install(TARGETS
  waypoint_recorder_node
  goal_selector_node
  trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
