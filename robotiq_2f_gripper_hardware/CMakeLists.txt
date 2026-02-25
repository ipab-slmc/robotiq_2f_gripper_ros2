cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_gripper_hardware LANGUAGES CXX)

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_2f_gripper_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_2f_gripper_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(robotiq_2f_gripper_node
  src/robotiq_2f_gripper_node.cpp
  src/config_loader.cpp
)
target_link_libraries(robotiq_2f_gripper_node PUBLIC
  ${robotiq_2f_gripper_interfaces_LIBRARIES}
  yaml-cpp
  ${robotiq_2f_gripper_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  robotiq_2f_gripper_interfaces::robotiq_2f_gripper_interfaces
  sensor_msgs::sensor_msgs_library
)

install(TARGETS robotiq_2f_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY 
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
