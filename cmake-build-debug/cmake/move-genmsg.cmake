# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "move: 7 messages, 0 services")

set(MSG_I_FLAGS "-Imove:/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg;-Irione_msgs:/home/migly/catkin_ws/src/rione_msgs/msg;-Iactionlib:/opt/ros/melodic/share/actionlib/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(move_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" "actionlib_msgs/GoalID:move/AmountActionResult:actionlib_msgs/GoalStatus:move/AmountResult:move/AmountActionGoal:rione_msgs/Velocity:move/AmountGoal:std_msgs/Header:move/AmountActionFeedback:rione_msgs/Amount:move/AmountFeedback"
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" ""
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" "rione_msgs/Amount:rione_msgs/Velocity"
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:move/AmountFeedback:rione_msgs/Amount:std_msgs/Header"
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" "move/AmountResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" "actionlib_msgs/GoalID:rione_msgs/Velocity:move/AmountGoal:rione_msgs/Amount:std_msgs/Header"
)

get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_custom_target(_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "move" "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" "rione_msgs/Amount"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)
_generate_msg_cpp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
)

### Generating Services

### Generating Module File
_generate_module_cpp(move
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(move_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(move_generate_messages move_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_cpp _move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_gencpp)
add_dependencies(move_gencpp move_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)
_generate_msg_eus(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
)

### Generating Services

### Generating Module File
_generate_module_eus(move
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(move_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(move_generate_messages move_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_eus _move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_geneus)
add_dependencies(move_geneus move_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)
_generate_msg_lisp(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
)

### Generating Services

### Generating Module File
_generate_module_lisp(move
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(move_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(move_generate_messages move_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_lisp _move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_genlisp)
add_dependencies(move_genlisp move_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)
_generate_msg_nodejs(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
)

### Generating Services

### Generating Module File
_generate_module_nodejs(move
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(move_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(move_generate_messages move_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_nodejs _move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_gennodejs)
add_dependencies(move_gennodejs move_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Velocity.msg;/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg;/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)
_generate_msg_py(move
  "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/migly/catkin_ws/src/rione_msgs/msg/Amount.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
)

### Generating Services

### Generating Module File
_generate_module_py(move
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(move_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(move_generate_messages move_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountAction.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountResult.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionResult.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountActionGoal.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/migly/catkin_ws/src/move/cmake-build-debug/devel/share/move/msg/AmountFeedback.msg" NAME_WE)
add_dependencies(move_generate_messages_py _move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(move_genpy)
add_dependencies(move_genpy move_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS move_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/move
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET rione_msgs_generate_messages_cpp)
  add_dependencies(move_generate_messages_cpp rione_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_generate_messages_cpp)
  add_dependencies(move_generate_messages_cpp actionlib_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(move_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/move
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET rione_msgs_generate_messages_eus)
  add_dependencies(move_generate_messages_eus rione_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_generate_messages_eus)
  add_dependencies(move_generate_messages_eus actionlib_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(move_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/move
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET rione_msgs_generate_messages_lisp)
  add_dependencies(move_generate_messages_lisp rione_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_generate_messages_lisp)
  add_dependencies(move_generate_messages_lisp actionlib_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(move_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/move
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET rione_msgs_generate_messages_nodejs)
  add_dependencies(move_generate_messages_nodejs rione_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_generate_messages_nodejs)
  add_dependencies(move_generate_messages_nodejs actionlib_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(move_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/move
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET rione_msgs_generate_messages_py)
  add_dependencies(move_generate_messages_py rione_msgs_generate_messages_py)
endif()
if(TARGET actionlib_generate_messages_py)
  add_dependencies(move_generate_messages_py actionlib_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(move_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
