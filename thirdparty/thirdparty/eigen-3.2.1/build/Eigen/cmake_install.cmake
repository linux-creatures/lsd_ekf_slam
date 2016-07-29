# Install script for directory: /home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen

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
   "/usr/local/include/eigen3/Eigen/StdDeque;/usr/local/include/eigen3/Eigen/Eigenvalues;/usr/local/include/eigen3/Eigen/PardisoSupport;/usr/local/include/eigen3/Eigen/Householder;/usr/local/include/eigen3/Eigen/MetisSupport;/usr/local/include/eigen3/Eigen/SparseCore;/usr/local/include/eigen3/Eigen/SparseLU;/usr/local/include/eigen3/Eigen/LeastSquares;/usr/local/include/eigen3/Eigen/OrderingMethods;/usr/local/include/eigen3/Eigen/SuperLUSupport;/usr/local/include/eigen3/Eigen/QR;/usr/local/include/eigen3/Eigen/StdList;/usr/local/include/eigen3/Eigen/SparseCholesky;/usr/local/include/eigen3/Eigen/SparseQR;/usr/local/include/eigen3/Eigen/Cholesky;/usr/local/include/eigen3/Eigen/Jacobi;/usr/local/include/eigen3/Eigen/Array;/usr/local/include/eigen3/Eigen/IterativeLinearSolvers;/usr/local/include/eigen3/Eigen/CholmodSupport;/usr/local/include/eigen3/Eigen/Dense;/usr/local/include/eigen3/Eigen/LU;/usr/local/include/eigen3/Eigen/QtAlignedMalloc;/usr/local/include/eigen3/Eigen/Geometry;/usr/local/include/eigen3/Eigen/Sparse;/usr/local/include/eigen3/Eigen/SVD;/usr/local/include/eigen3/Eigen/Eigen;/usr/local/include/eigen3/Eigen/SPQRSupport;/usr/local/include/eigen3/Eigen/PaStiXSupport;/usr/local/include/eigen3/Eigen/Core;/usr/local/include/eigen3/Eigen/StdVector;/usr/local/include/eigen3/Eigen/Eigen2Support;/usr/local/include/eigen3/Eigen/UmfPackSupport")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/local/include/eigen3/Eigen" TYPE FILE FILES
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/StdDeque"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Eigenvalues"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/PardisoSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Householder"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/MetisSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SparseCore"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SparseLU"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/LeastSquares"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/OrderingMethods"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SuperLUSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/QR"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/StdList"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SparseCholesky"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SparseQR"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Cholesky"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Jacobi"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Array"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/IterativeLinearSolvers"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/CholmodSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Dense"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/LU"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/QtAlignedMalloc"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Geometry"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Sparse"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SVD"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Eigen"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/SPQRSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/PaStiXSupport"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Core"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/StdVector"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/Eigen2Support"
    "/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/Eigen/UmfPackSupport"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/kiran/catkin_ws/src/ardrone_navmap/thirdparty/thirdparty/eigen-3.2.1/build/Eigen/src/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

