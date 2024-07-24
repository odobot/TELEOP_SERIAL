cmake_minimum_required(VERSION 3.8)
project(cpp_code_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()  

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(Serial_node src/ROS_STM32_SERIAL_NODE.cpp)
ament_target_dependencies(Serial_node rclcpp)
ament_target_dependencies(Serial_node rclcpp std_msgs)

add_executable(sub_Node src/subscriber_template.cpp)
ament_target_dependencies(sub_Node rclcpp example_interfaces)

add_executable(Teleop_STM src/TELEOP_STM_Control.cpp)
ament_target_dependencies(Teleop_STM rclcpp)
ament_target_dependencies(Teleop_STM rclcpp std_msgs)
ament_target_dependencies(Teleop_STM rclcpp geometry_msgs)


install(TARGETS
Serial_node
sub_Node
Teleop_STM
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
