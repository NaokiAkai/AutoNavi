cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  GetPath.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_follower
#  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include
  /usr/local/include
  /usr/local/include/eigen3
)

add_executable(path_server src/path_server.cpp)
target_link_libraries(path_server ${catkin_LIBRARIES})
add_dependencies(path_server path_follower_gencpp)

add_executable(path_recorder src/path_recorder.cpp)
target_link_libraries(path_recorder ${catkin_LIBRARIES})

add_executable(path_saver src/path_saver.cpp)
target_link_libraries(path_saver ${catkin_LIBRARIES})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})

add_executable(simple_obstacle_avoidance src/simple_obstacle_avoidance.cpp)
target_link_libraries(simple_obstacle_avoidance ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
