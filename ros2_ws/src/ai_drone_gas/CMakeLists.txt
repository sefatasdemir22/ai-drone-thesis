cmake_minimum_required(VERSION 3.10)
project(ai_drone_gas)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Gazebo C++ Plugin ---
add_library(gas_field_plugin SHARED
  legacy_plugin/gas_field_plugin.cpp
)

target_include_directories(gas_field_plugin PUBLIC
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(gas_field_plugin
  ${GAZEBO_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(gas_field_plugin
  rclcpp
  std_msgs
)

# --- Python Script Kurulumu ---
install(PROGRAMS
  scripts/sim_gas_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- C++ Plugin Kurulumu ---
install(TARGETS gas_field_plugin
  LIBRARY DESTINATION lib
)

ament_package()

