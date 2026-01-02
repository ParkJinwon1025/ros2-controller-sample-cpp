cmake_minimum_required(VERSION 3.8)
project(test_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(test_interfaces REQUIRED)

add_executable(node_a src/node_a.cpp)
ament_target_dependencies(node_a rclcpp rclcpp_action test_interfaces)

add_executable(node_b src/node_b.cpp)
ament_target_dependencies(node_b rclcpp rclcpp_action test_interfaces)

install(TARGETS
  node_a
  node_b
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()