cmake_minimum_required(VERSION 3.20)
project(robot_simulation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_business REQUIRED)
find_package(robot_interfaces_facility REQUIRED)
find_package(robot_interfaces_fleet REQUIRED)
find_package(robot_interfaces_mission REQUIRED)
find_package(robot_interfaces_navigation REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(amr_sim_visualizer_node src/amr_sim_visualizer_node.cpp)
ament_target_dependencies(amr_sim_visualizer_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_business
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  std_msgs
  visualization_msgs)

add_executable(amr_sim_demo_director_node src/amr_sim_demo_director_node.cpp)
ament_target_dependencies(amr_sim_demo_director_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_business
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  robot_interfaces_navigation
  std_msgs)

add_executable(amr_sim_gazebo_entity_bridge_node src/amr_sim_gazebo_entity_bridge_node.cpp)
ament_target_dependencies(amr_sim_gazebo_entity_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  ros_gz_interfaces
  std_msgs)

add_executable(robot2_fleet_demo_driver_node src/robot2_fleet_demo_driver_node.cpp)
ament_target_dependencies(robot2_fleet_demo_driver_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_fleet
  robot_interfaces_mission)

add_executable(gazebo_odom_tf_node src/gazebo_odom_tf_node.cpp)
ament_target_dependencies(gazebo_odom_tf_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros)

add_executable(gazebo_unpause_node src/gazebo_unpause_node.cpp)
ament_target_dependencies(gazebo_unpause_node
  rclcpp
  ros_gz_interfaces)

install(TARGETS
  amr_sim_visualizer_node
  amr_sim_demo_director_node
  amr_sim_gazebo_entity_bridge_node
  robot2_fleet_demo_driver_node
  gazebo_odom_tf_node
  gazebo_unpause_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch media models rviz worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
