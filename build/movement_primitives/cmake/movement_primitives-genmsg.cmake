# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "movement_primitives: 5 messages, 1 services")

set(MSG_I_FLAGS "-Imovement_primitives:/home/abril/catkin_ws/src/movement-primitives/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(movement_primitives_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" ""
)

get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" ""
)

get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" ""
)

get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" ""
)

get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" ""
)

get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_custom_target(_movement_primitives_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "movement_primitives" "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" "std_msgs/UInt32MultiArray:std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)
_generate_msg_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)
_generate_msg_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)
_generate_msg_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)
_generate_msg_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)

### Generating Services
_generate_srv_cpp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
)

### Generating Module File
_generate_module_cpp(movement_primitives
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(movement_primitives_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(movement_primitives_generate_messages movement_primitives_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_dependencies(movement_primitives_generate_messages_cpp _movement_primitives_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movement_primitives_gencpp)
add_dependencies(movement_primitives_gencpp movement_primitives_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movement_primitives_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)
_generate_msg_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)
_generate_msg_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)
_generate_msg_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)
_generate_msg_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)

### Generating Services
_generate_srv_eus(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
)

### Generating Module File
_generate_module_eus(movement_primitives
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(movement_primitives_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(movement_primitives_generate_messages movement_primitives_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_dependencies(movement_primitives_generate_messages_eus _movement_primitives_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movement_primitives_geneus)
add_dependencies(movement_primitives_geneus movement_primitives_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movement_primitives_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)
_generate_msg_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)
_generate_msg_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)
_generate_msg_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)
_generate_msg_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)

### Generating Services
_generate_srv_lisp(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
)

### Generating Module File
_generate_module_lisp(movement_primitives
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(movement_primitives_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(movement_primitives_generate_messages movement_primitives_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_dependencies(movement_primitives_generate_messages_lisp _movement_primitives_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movement_primitives_genlisp)
add_dependencies(movement_primitives_genlisp movement_primitives_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movement_primitives_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)
_generate_msg_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)
_generate_msg_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)
_generate_msg_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)
_generate_msg_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)

### Generating Services
_generate_srv_nodejs(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
)

### Generating Module File
_generate_module_nodejs(movement_primitives
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(movement_primitives_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(movement_primitives_generate_messages movement_primitives_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_dependencies(movement_primitives_generate_messages_nodejs _movement_primitives_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movement_primitives_gennodejs)
add_dependencies(movement_primitives_gennodejs movement_primitives_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movement_primitives_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)
_generate_msg_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)
_generate_msg_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)
_generate_msg_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)
_generate_msg_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)

### Generating Services
_generate_srv_py(movement_primitives
  "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
)

### Generating Module File
_generate_module_py(movement_primitives
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(movement_primitives_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(movement_primitives_generate_messages movement_primitives_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/PrompQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/RobotTraj.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMTrainMsg.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/msg/TPHSMMQueryTrigger.msg" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abril/catkin_ws/src/movement-primitives/srv/PrompTrain.srv" NAME_WE)
add_dependencies(movement_primitives_generate_messages_py _movement_primitives_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(movement_primitives_genpy)
add_dependencies(movement_primitives_genpy movement_primitives_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS movement_primitives_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/movement_primitives
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(movement_primitives_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/movement_primitives
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(movement_primitives_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/movement_primitives
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(movement_primitives_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/movement_primitives
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(movement_primitives_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/movement_primitives
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(movement_primitives_generate_messages_py std_msgs_generate_messages_py)
endif()
