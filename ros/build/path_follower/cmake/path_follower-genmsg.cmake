# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "path_follower: 0 messages, 1 services")

set(MSG_I_FLAGS "-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(path_follower_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv" NAME_WE)
add_custom_target(_path_follower_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "path_follower" "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:nav_msgs/Path:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(path_follower
  "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/path_follower
)

### Generating Module File
_generate_module_cpp(path_follower
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/path_follower
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(path_follower_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(path_follower_generate_messages path_follower_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv" NAME_WE)
add_dependencies(path_follower_generate_messages_cpp _path_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(path_follower_gencpp)
add_dependencies(path_follower_gencpp path_follower_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS path_follower_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(path_follower
  "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/path_follower
)

### Generating Module File
_generate_module_lisp(path_follower
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/path_follower
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(path_follower_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(path_follower_generate_messages path_follower_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv" NAME_WE)
add_dependencies(path_follower_generate_messages_lisp _path_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(path_follower_genlisp)
add_dependencies(path_follower_genlisp path_follower_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS path_follower_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(path_follower
  "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/path_follower
)

### Generating Module File
_generate_module_py(path_follower
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/path_follower
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(path_follower_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(path_follower_generate_messages path_follower_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/akai/Dropbox/work/AutoNavi/ros/src/path_follower/srv/GetPath.srv" NAME_WE)
add_dependencies(path_follower_generate_messages_py _path_follower_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(path_follower_genpy)
add_dependencies(path_follower_genpy path_follower_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS path_follower_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/path_follower)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/path_follower
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(path_follower_generate_messages_cpp nav_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/path_follower)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/path_follower
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(path_follower_generate_messages_lisp nav_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/path_follower)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/path_follower\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/path_follower
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(path_follower_generate_messages_py nav_msgs_generate_messages_py)
