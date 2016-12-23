# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nav_common: 1 messages, 0 services")

set(MSG_I_FLAGS "-Inav_common:/home/bart/workspace/AERO/src/nav_common/msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nav_common_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_custom_target(_nav_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nav_common" "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nav_common
  "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nav_common
)

### Generating Services

### Generating Module File
_generate_module_cpp(nav_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nav_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nav_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nav_common_generate_messages nav_common_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_dependencies(nav_common_generate_messages_cpp _nav_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nav_common_gencpp)
add_dependencies(nav_common_gencpp nav_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nav_common_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nav_common
  "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nav_common
)

### Generating Services

### Generating Module File
_generate_module_eus(nav_common
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nav_common
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nav_common_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nav_common_generate_messages nav_common_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_dependencies(nav_common_generate_messages_eus _nav_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nav_common_geneus)
add_dependencies(nav_common_geneus nav_common_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nav_common_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nav_common
  "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nav_common
)

### Generating Services

### Generating Module File
_generate_module_lisp(nav_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nav_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nav_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nav_common_generate_messages nav_common_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_dependencies(nav_common_generate_messages_lisp _nav_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nav_common_genlisp)
add_dependencies(nav_common_genlisp nav_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nav_common_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nav_common
  "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nav_common
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nav_common
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nav_common
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nav_common_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nav_common_generate_messages nav_common_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_dependencies(nav_common_generate_messages_nodejs _nav_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nav_common_gennodejs)
add_dependencies(nav_common_gennodejs nav_common_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nav_common_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nav_common
  "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nav_common
)

### Generating Services

### Generating Module File
_generate_module_py(nav_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nav_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nav_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nav_common_generate_messages nav_common_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bart/workspace/AERO/src/nav_common/msg/movement_request.msg" NAME_WE)
add_dependencies(nav_common_generate_messages_py _nav_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nav_common_genpy)
add_dependencies(nav_common_genpy nav_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nav_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nav_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nav_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(nav_common_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(nav_common_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nav_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nav_common
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(nav_common_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(nav_common_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nav_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nav_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(nav_common_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(nav_common_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nav_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nav_common
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(nav_common_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(nav_common_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nav_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nav_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nav_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(nav_common_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(nav_common_generate_messages_py std_msgs_generate_messages_py)
endif()
