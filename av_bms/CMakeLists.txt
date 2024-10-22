cmake_minimum_required(VERSION 3.8)
project(av_bms)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(bms_node_component SHARED src/bms_node.cpp)

target_link_libraries(bms_node_component ${rclcpp_LIBRARIES})

rclcpp_components_register_node(bms_node_component PLUGIN "bms::BMS_node"
                                EXECUTABLE bms_node)

ament_target_dependencies(bms_node_component can_msgs sensor_msgs rclcpp
                          rclcpp_components)

ament_export_targets(export_bms_node_component)
install(
  TARGETS bms_node_component
  EXPORT export_bms_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
