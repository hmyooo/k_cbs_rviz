# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dy_avoidobs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Idy_avoidobs:/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dy_avoidobs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_custom_target(_dy_avoidobs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dy_avoidobs" "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_custom_target(_dy_avoidobs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dy_avoidobs" "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" "geometry_msgs/Point:dy_avoidobs/localtraj"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dy_avoidobs
)
_generate_msg_cpp(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dy_avoidobs
)

### Generating Services

### Generating Module File
_generate_module_cpp(dy_avoidobs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dy_avoidobs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dy_avoidobs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dy_avoidobs_generate_messages dy_avoidobs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_cpp _dy_avoidobs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_cpp _dy_avoidobs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dy_avoidobs_gencpp)
add_dependencies(dy_avoidobs_gencpp dy_avoidobs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dy_avoidobs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dy_avoidobs
)
_generate_msg_eus(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dy_avoidobs
)

### Generating Services

### Generating Module File
_generate_module_eus(dy_avoidobs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dy_avoidobs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dy_avoidobs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dy_avoidobs_generate_messages dy_avoidobs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_eus _dy_avoidobs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_eus _dy_avoidobs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dy_avoidobs_geneus)
add_dependencies(dy_avoidobs_geneus dy_avoidobs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dy_avoidobs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dy_avoidobs
)
_generate_msg_lisp(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dy_avoidobs
)

### Generating Services

### Generating Module File
_generate_module_lisp(dy_avoidobs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dy_avoidobs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dy_avoidobs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dy_avoidobs_generate_messages dy_avoidobs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_lisp _dy_avoidobs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_lisp _dy_avoidobs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dy_avoidobs_genlisp)
add_dependencies(dy_avoidobs_genlisp dy_avoidobs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dy_avoidobs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dy_avoidobs
)
_generate_msg_nodejs(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dy_avoidobs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dy_avoidobs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dy_avoidobs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dy_avoidobs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dy_avoidobs_generate_messages dy_avoidobs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_nodejs _dy_avoidobs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_nodejs _dy_avoidobs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dy_avoidobs_gennodejs)
add_dependencies(dy_avoidobs_gennodejs dy_avoidobs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dy_avoidobs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs
)
_generate_msg_py(dy_avoidobs
  "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs
)

### Generating Services

### Generating Module File
_generate_module_py(dy_avoidobs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dy_avoidobs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dy_avoidobs_generate_messages dy_avoidobs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/localtraj.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_py _dy_avoidobs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rancho/1hmy/k_cbs_swarmrviz/src/src/src/src/dy_avoidobs/msg/Multilocaltrajs.msg" NAME_WE)
add_dependencies(dy_avoidobs_generate_messages_py _dy_avoidobs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dy_avoidobs_genpy)
add_dependencies(dy_avoidobs_genpy dy_avoidobs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dy_avoidobs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dy_avoidobs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dy_avoidobs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dy_avoidobs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dy_avoidobs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dy_avoidobs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dy_avoidobs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dy_avoidobs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dy_avoidobs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dy_avoidobs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dy_avoidobs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dy_avoidobs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dy_avoidobs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dy_avoidobs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dy_avoidobs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dy_avoidobs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dy_avoidobs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dy_avoidobs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dy_avoidobs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dy_avoidobs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
