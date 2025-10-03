cmake_minimum_required(VERSION 3.10)
project(ai_drone_gas)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)


include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})


add_library(gas_field_plugin SHARED src/gas_field_plugin.cpp)
ament_target_dependencies(gas_field_plugin rclcpp std_msgs)
target_link_libraries(gas_field_plugin ${GAZEBO_LIBRARIES})


install(TARGETS gas_field_plugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)


ament_package()