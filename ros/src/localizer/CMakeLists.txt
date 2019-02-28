cmake_minimum_required(VERSION 2.8.3)
project(localizer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  grid_map_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localizer
#  CATKIN_DEPENDS cv_bridge message_generation pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
#  /usr/local/include
)

if(UNIX)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
endif()

add_executable(amcl src/amcl_node.cpp src/amcl.cpp)
target_link_libraries(amcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(amcl robot_sim_gencpp)

add_executable(ndt_mapping src/ndt_mapping.cpp src/ndt.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})

add_executable(ndt_amcl src/ndt_amcl_node.cpp src/amcl.cpp src/ndt.cpp)
target_link_libraries(ndt_amcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(gyro_odom src/gyro_odom.cpp)
target_link_libraries(gyro_odom ${catkin_LIBRARIES})

