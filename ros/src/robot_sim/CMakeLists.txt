cmake_minimum_required(VERSION 2.8.3)
project(robot_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  pcl_conversions
)

add_message_files(
  FILES
  ScanObjectID.msg
  SemanticScan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_sim
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  "/usr/local/include"
)

add_executable(odom_sim src/odom_sim.cpp)
target_link_libraries(odom_sim ${catkin_LIBRARIES})

add_executable(semantic_map_publisher src/semantic_map_publisher.cpp)
target_link_libraries(semantic_map_publisher ${catkin_LIBRARIES} yaml-cpp SDL SDL_image)

add_executable(laser_sim src/laser_sim.cpp)
target_link_libraries(laser_sim ${catkin_LIBRARIES})
add_dependencies(laser_sim robot_sim_gencpp)

add_executable(key_to_twist_cmd src/key_to_twist_cmd.cpp)
target_link_libraries(key_to_twist_cmd ${catkin_LIBRARIES})

add_executable(joy_to_twist_cmd src/joy_to_twist_cmd.cpp)
target_link_libraries(joy_to_twist_cmd ${catkin_LIBRARIES})

add_executable(velocity_selector src/velocity_selector.cpp)
target_link_libraries(velocity_selector ${catkin_LIBRARIES})

add_executable(add_noise_to_sensor_data src/add_noise_to_sensor_data.cpp)
target_link_libraries(add_noise_to_sensor_data ${catkin_LIBRARIES})
add_dependencies(add_noise_to_sensor_data robot_sim_gencpp)

add_executable(robot_sim_recorder src/robot_sim_recorder.cpp)
target_link_libraries(robot_sim_recorder ${catkin_LIBRARIES})
