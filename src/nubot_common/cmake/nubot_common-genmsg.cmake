# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nubot_common: 17 messages, 2 services")

set(MSG_I_FLAGS "-Inubot_common:/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nubot_common_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg" "nubot_common/Point2d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg" "nubot_common/Angle:std_msgs/Header:nubot_common/Point2d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg" "nubot_common/Point3d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg" "nubot_common/ObstaclesInfo:nubot_common/RobotInfo:nubot_common/Point2d:nubot_common/PPoint:std_msgs/Header:nubot_common/BallInfo:nubot_common/CoachInfo:nubot_common/Angle"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg" "nubot_common/PPoint:std_msgs/Header:nubot_common/Point2d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg" "nubot_common/Point3d:std_msgs/Header"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg" "nubot_common/PPoint:nubot_common/RobotInfo:nubot_common/Point2d:nubot_common/ObstaclesInfo:std_msgs/Header:nubot_common/BallInfo:nubot_common/Angle"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg" "nubot_common/PPoint:std_msgs/Header:nubot_common/Point2d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg" "std_msgs/Header:nubot_common/Point2d"
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv" ""
)

get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg" NAME_WE)
add_custom_target(_nubot_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nubot_common" "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg" "nubot_common/PPoint:std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_msg_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)

### Generating Services
_generate_srv_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)
_generate_srv_cpp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
)

### Generating Module File
_generate_module_cpp(nubot_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nubot_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nubot_common_generate_messages nubot_common_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_cpp _nubot_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nubot_common_gencpp)
add_dependencies(nubot_common_gencpp nubot_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nubot_common_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_msg_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)

### Generating Services
_generate_srv_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)
_generate_srv_lisp(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
)

### Generating Module File
_generate_module_lisp(nubot_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nubot_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nubot_common_generate_messages nubot_common_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_lisp _nubot_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nubot_common_genlisp)
add_dependencies(nubot_common_genlisp nubot_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nubot_common_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_msg_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)

### Generating Services
_generate_srv_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)
_generate_srv_py(nubot_common
  "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
)

### Generating Module File
_generate_module_py(nubot_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nubot_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nubot_common_generate_messages nubot_common_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg" NAME_WE)
add_dependencies(nubot_common_generate_messages_py _nubot_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nubot_common_genpy)
add_dependencies(nubot_common_genpy nubot_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nubot_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nubot_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(nubot_common_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nubot_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(nubot_common_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nubot_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(nubot_common_generate_messages_py std_msgs_generate_messages_py)
