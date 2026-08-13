cmake_minimum_required(VERSION 3.20)
project(robot_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces_perception REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(depth_projector src/depth_projector.cpp)
target_include_directories(depth_projector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(depth_projector geometry_msgs sensor_msgs)

add_library(target_manager src/target_manager.cpp)
target_include_directories(target_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(geometry_validation_node src/geometry_validation_node.cpp)
target_link_libraries(geometry_validation_node depth_projector target_manager)
ament_target_dependencies(geometry_validation_node
  geometry_msgs
  builtin_interfaces
  message_filters
  rclcpp
  robot_interfaces_perception
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  vision_msgs)

install(TARGETS depth_projector target_manager geometry_validation_node
  EXPORT export_robot_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/detector_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(depth_projector_test test/depth_projector_test.cpp)
  target_link_libraries(depth_projector_test depth_projector)
  ament_target_dependencies(depth_projector_test geometry_msgs sensor_msgs)
  ament_add_gtest(target_manager_test test/target_manager_test.cpp)
  target_link_libraries(target_manager_test target_manager)
  ament_add_pytest_test(detection_utils_test test/test_detection_utils.py)
endif()

ament_export_targets(export_robot_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces geometry_msgs robot_interfaces_perception sensor_msgs vision_msgs)
ament_package()
