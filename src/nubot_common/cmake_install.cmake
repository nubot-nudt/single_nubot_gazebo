# Install script for directory: /home/winston/workspace/nubot_ws/src/nubot/nubot_common

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common/msgs" TYPE FILE FILES
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Angle.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point2d.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/PPoint.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/Point3d.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/RobotInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/MotorInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/BallInfo3d.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/ObstaclesInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OminiVisionInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/VelCmd.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/OdoInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/CoachInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/WorldModelInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/StrategyInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/TargetInfo.msg"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/msgs/FrontBallInfo.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common/srv" TYPE FILE FILES
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/BallHandle.srv"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/srv/Shoot.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common/cmake" TYPE FILE FILES "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/catkin_generated/installspace/nubot_common-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/winston/workspace/nubot_ws/src/devel/include/nubot_common")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/winston/workspace/nubot_ws/src/devel/share/common-lisp/ros/nubot_common")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/winston/workspace/nubot_ws/src/devel/lib/python2.7/dist-packages/nubot_common")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/winston/workspace/nubot_ws/src/devel/lib/python2.7/dist-packages/nubot_common")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/catkin_generated/installspace/nubot_common.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common/cmake" TYPE FILE FILES "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/catkin_generated/installspace/nubot_common-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common/cmake" TYPE FILE FILES
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/catkin_generated/installspace/nubot_commonConfig.cmake"
    "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/catkin_generated/installspace/nubot_commonConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nubot_common" TYPE FILE FILES "/home/winston/workspace/nubot_ws/src/nubot/nubot_common/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

