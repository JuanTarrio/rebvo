# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rebvo: 2 messages, 0 services")

set(MSG_I_FLAGS "-Irebvo:/home/juan/git/rebvo/ros/src/rebvo_ros/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rebvo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg" NAME_WE)
add_custom_target(_rebvo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rebvo" "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg" ""
)

get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg" NAME_WE)
add_custom_target(_rebvo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rebvo" "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg" "geometry_msgs/Point:rebvo/Keyline:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rebvo
)
_generate_msg_cpp(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rebvo
)

### Generating Services

### Generating Module File
_generate_module_cpp(rebvo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rebvo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rebvo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rebvo_generate_messages rebvo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_cpp _rebvo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_cpp _rebvo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rebvo_gencpp)
add_dependencies(rebvo_gencpp rebvo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rebvo_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rebvo
)
_generate_msg_lisp(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rebvo
)

### Generating Services

### Generating Module File
_generate_module_lisp(rebvo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rebvo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rebvo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rebvo_generate_messages rebvo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_lisp _rebvo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_lisp _rebvo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rebvo_genlisp)
add_dependencies(rebvo_genlisp rebvo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rebvo_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo
)
_generate_msg_py(rebvo
  "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo
)

### Generating Services

### Generating Module File
_generate_module_py(rebvo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rebvo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rebvo_generate_messages rebvo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/Keyline.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_py _rebvo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juan/git/rebvo/ros/src/rebvo_ros/msg/EdgeMap.msg" NAME_WE)
add_dependencies(rebvo_generate_messages_py _rebvo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rebvo_genpy)
add_dependencies(rebvo_genpy rebvo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rebvo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rebvo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rebvo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(rebvo_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(rebvo_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rebvo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rebvo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(rebvo_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(rebvo_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rebvo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(rebvo_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(rebvo_generate_messages_py geometry_msgs_generate_messages_py)
