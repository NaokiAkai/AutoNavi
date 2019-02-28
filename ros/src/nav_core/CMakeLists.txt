cmake_minimum_required(VERSION 2.8.3)
project(nav_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_core
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(nav_core_path_register src/nav_core_path_register.cpp)
target_link_libraries(nav_core_path_register ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nav_core_server src/nav_core_server.cpp)
target_link_libraries(nav_core_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
