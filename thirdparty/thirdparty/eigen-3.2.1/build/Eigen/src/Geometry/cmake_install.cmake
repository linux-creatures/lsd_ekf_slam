# Install script for directory: /home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry

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
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
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

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/eigen3/Eigen/src/Geometry/Hyperplane.h;/usr/local/include/eigen3/Eigen/src/Geometry/Translation.h;/usr/local/include/eigen3/Eigen/src/Geometry/Quaternion.h;/usr/local/include/eigen3/Eigen/src/Geometry/Homogeneous.h;/usr/local/include/eigen3/Eigen/src/Geometry/EulerAngles.h;/usr/local/include/eigen3/Eigen/src/Geometry/RotationBase.h;/usr/local/include/eigen3/Eigen/src/Geometry/AlignedBox.h;/usr/local/include/eigen3/Eigen/src/Geometry/Rotation2D.h;/usr/local/include/eigen3/Eigen/src/Geometry/Umeyama.h;/usr/local/include/eigen3/Eigen/src/Geometry/AngleAxis.h;/usr/local/include/eigen3/Eigen/src/Geometry/Transform.h;/usr/local/include/eigen3/Eigen/src/Geometry/Scaling.h;/usr/local/include/eigen3/Eigen/src/Geometry/ParametrizedLine.h;/usr/local/include/eigen3/Eigen/src/Geometry/OrthoMethods.h")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/local/include/eigen3/Eigen/src/Geometry" TYPE FILE FILES
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Hyperplane.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Translation.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Quaternion.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Homogeneous.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/EulerAngles.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/RotationBase.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/AlignedBox.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Rotation2D.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Umeyama.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/AngleAxis.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Transform.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/Scaling.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/ParametrizedLine.h"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/src/Geometry/OrthoMethods.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/build/Eigen/src/Geometry/arch/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

