#
# Author: Jakob Engel <jajuengel@gmail.com>
# Contributor: Stefan Wilkes <stefan.wilkes@gmail.com>
#
cmake_minimum_required(VERSION 2.8.3)
project(ardrone_navmap)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  roscpp
  rospy
)

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdparty
	URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
	PREFIX ${CMAKE_BINARY_DIR}/thirdparty
	CONFIGURE_COMMAND ""
	BUILD_COMMAND make
	INSTALL_COMMAND ""
	BUILD_IN_SOURCE 1
)

# ------------------- add dynamic reconfigure api ------------------------------------
generate_dynamic_reconfigure_options(
  cfg/AutopilotParams.cfg
  cfg/GUIParams.cfg
  cfg/StateestimationParams.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES filter_state.msg)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	SetReference.srv
	SetMaxControl.srv
	SetInitialReachDistance.srv
	SetStayWithinDistance.srv
	SetStayTime.srv
)

## Generate added messages 
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# --------------------------- stateestimation & PTAM --------------------------------
# set header ans source files
set(STATEESTIMATION_SOURCE_FILES       
  src/stateestimation/LSD-SLAM/util/undistorter.cc
  src/stateestimation/LSD-SLAM/util/sophus_util.cc
  src/stateestimation/LSD-SLAM/util/settings.cc
  src/stateestimation/LSD-SLAM/util/global_funcs.cc
  src/stateestimation/LSD-SLAM/tracking/tracking_reference.cc
  src/stateestimation/LSD-SLAM/tracking/sim3_tracker.cc
  src/stateestimation/LSD-SLAM/tracking/se3_tracker.cc
  src/stateestimation/LSD-SLAM/tracking/relocalizer.cc
  src/stateestimation/LSD-SLAM/tracking/least_squares.cc
  src/stateestimation/LSD-SLAM/slam_system.cc
  src/stateestimation/LSD-SLAM/model/frame_pose_struct.cc
  src/stateestimation/LSD-SLAM/model/frame_memory.cc
  src/stateestimation/LSD-SLAM/model/frame.cc
  src/stateestimation/LSD-SLAM/io_wrapper/timestamp.cc
  src/stateestimation/LSD-SLAM/io_wrapper/ImageDisplay_OpenCV.cc
  src/stateestimation/LSD-SLAM/global_mapping/trackable_key_frame_search.cc
  src/stateestimation/LSD-SLAM/global_mapping/key_frame_graph.cc
  src/stateestimation/LSD-SLAM/global_mapping/g2o_type_sim3_sophus.cc
  src/stateestimation/LSD-SLAM/global_mapping/fab_map.cc
  src/stateestimation/LSD-SLAM/depth_estimation/depth_map_pixel_hypothesis.cc
  src/stateestimation/LSD-SLAM/depth_estimation/depth_map.cc
  src/stateestimation/GLWindowMenu.cc
  src/stateestimation/GLWindow2.cc
  src/stateestimation/Predictor.cpp
  src/stateestimation/MapView.cpp
  src/stateestimation/main_stateestimation.cpp
  src/stateestimation/LSDWrapper.cpp
  src/stateestimation/LSD-SLAM/MapView.cpp
  src/stateestimation/LSD-SLAM/io_wrapper/ROS/ROSOutput3DWrapper.cpp
  src/stateestimation/LSD-SLAM/io_wrapper/ROS/ROSImageStreamThread.cpp
  src/stateestimation/LSD-SLAM/io_wrapper/OpenCVImageStreamThread.cpp
  src/stateestimation/EstimationNode.cpp
  src/stateestimation/DroneKalmanFilter.cpp


)
set(STATEESTIMATION_HEADER_FILES   
  src/stateestimation/Predictor.h
  src/stateestimation/MouseKeyHandler.h
  src/stateestimation/MapView.h
  src/stateestimation/LSDWrapper.h
  src/stateestimation/LSD-SLAM/util/undistorter.h
  src/stateestimation/LSD-SLAM/util/sophus_util.h
  src/stateestimation/LSD-SLAM/util/sophus_util.cc
  src/stateestimation/LSD-SLAM/util/snprintf.h
  src/stateestimation/LSD-SLAM/util/settings.h
  src/stateestimation/LSD-SLAM/util/index_thread_reduce.h
  src/stateestimation/LSD-SLAM/util/global_funcs.h
  src/stateestimation/LSD-SLAM/util/eigen_core_include.h
  src/stateestimation/LSD-SLAM/tracking/tracking_reference.h
  src/stateestimation/LSD-SLAM/tracking/sim3_tracker.h
  src/stateestimation/LSD-SLAM/tracking/se3_tracker.h
  src/stateestimation/LSD-SLAM/tracking/relocalizer.h
  src/stateestimation/LSD-SLAM/tracking/least_squares.h
  src/stateestimation/LSD-SLAM/slam_system.h
  src/stateestimation/LSD-SLAM/OpenGL.h
  src/stateestimation/LSD-SLAM/model/frame_pose_struct.h
  src/stateestimation/LSD-SLAM/model/frame_memory.h
  src/stateestimation/LSD-SLAM/model/frame.h
  src/stateestimation/LSD-SLAM/MapView.h
  src/stateestimation/LSD-SLAM/io_wrapper/timestamp.h
  src/stateestimation/LSD-SLAM/io_wrapper/timestamped_object.h
  src/stateestimation/LSD-SLAM/io_wrapper/ROS/rosReconfigure.h
  src/stateestimation/LSD-SLAM/io_wrapper/ROS/ROSOutput3DWrapper.h
  src/stateestimation/LSD-SLAM/io_wrapper/ROS/ROSImageStreamThread.h
  src/stateestimation/LSD-SLAM/io_wrapper/output_3d_wrapper.h
  src/stateestimation/LSD-SLAM/io_wrapper/OpenCVImageStreamThread.h
  src/stateestimation/LSD-SLAM/io_wrapper/notify_buffer.h
  src/stateestimation/LSD-SLAM/io_wrapper/input_image_stream.h
  src/stateestimation/LSD-SLAM/io_wrapper/image_display.h
  src/stateestimation/LSD-SLAM/global_mapping/trackable_key_frame_search.h
  src/stateestimation/LSD-SLAM/global_mapping/key_frame_graph.h
  src/stateestimation/LSD-SLAM/global_mapping/g2o_type_sim3_sophus.h
  src/stateestimation/LSD-SLAM/global_mapping/fab_map.h
  src/stateestimation/LSD-SLAM/depth_estimation/depth_map_pixel_hypothesis.h
  src/stateestimation/LSD-SLAM/depth_estimation/depth_map.h
  src/stateestimation/GLWindowMenu.h
  src/stateestimation/GLWindow2.h
  src/stateestimation/EstimationNode.h
  src/stateestimation/DroneKalmanFilter.h

)


# set required libs and headers
include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/Sophus/
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/sophus/
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/eigen-3.2.1/build/Eigen/src

)  

if(NOT ANDROID)  # no nonfree-feature2d in opencv android build yet!!!
  # FabMap
  include_directories(${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/openFabMap/include)
  add_definitions("-DHAVE_FABMAP")
  set(FABMAP_LIB openFABMAP )  # no need, only *.dll generated
endif()

if(WIN32)
  # Direct API
  include_directories("$${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/dirent-1.20.1/include")
endif()


if(ANDROID)
  add_definitions(-DENABLE_NEON)
else()
  add_definitions(-DENABLE_SSE)
  set(CMAKE_CXX_RELEASE_FLAGS ${CMAKE_CXX_RELEASE_FLAGS} ${SSE_FLAGS})
endif()


link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib 
)
set(PTAM_LIBRARIES GL glut cvd GVars3 blas lapack)
add_definitions(-DKF_REPROJ)




# build!
add_executable(drone_stateestimation ${STATEESTIMATION_SOURCE_FILES} ${STATEESTIMATION_HEADER_FILES})
set_target_properties(drone_stateestimation PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3") 
target_link_libraries(drone_stateestimation ${PTAM_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ------------------------- autopilot & KI -----------------------------------------
# set header ans source files
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/main_autopilot.cpp  
	src/autopilot/ControlNode.cpp
	src/autopilot/DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/ControlNode.h
	src/autopilot/DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIProcedure.h
)

# build!
add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})
add_dependencies(drone_autopilot thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ---------------------------- GUI --------------------------------------------------
# set header ans source files
set(GUI_SOURCE_FILES         
	src/UINode/main_GUI.cpp  
	src/UINode/tum_ardrone_gui.cpp  
	src/UINode/RosThread.cpp  
	src/UINode/PingThread.cpp  
)

set(GUI_HEADER_FILES        
	src/UINode/tum_ardrone_gui.h 
	src/UINode/RosThread.h  
	src/UINode/PingThread.h  
)

# *.ui
set(GUI_UI_FILES        
	src/UINode/tum_ardrone_gui.ui 
)

# *.qrc
set(GUI_RESOURCE_FILES        
)

# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# build!
add_executable(drone_gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(drone_gui ${QT_LIBRARIES} cvd ${catkin_LIBRARIES})
add_dependencies(drone_gui thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


