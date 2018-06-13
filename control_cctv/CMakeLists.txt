cmake_minimum_required(VERSION 2.8.3)
project(control_cctv)

set(CMAKE_CXX_STANDARD 11)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	quadrotor_msgs
   geometry_msgs
	std_msgs
   message_generation
	tf
	nodelet
	cmake_modules)

find_package(Eigen REQUIRED)

include_directories(include
	${catkin_INCLUDE_DIRS}
   ${EIGEN_INCLUDE_DIR})

add_message_files(DIRECTORY msg
   FILES
   PayloadCommand.msg
   test_msg.msg
   )

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ControlCableVision
  CATKIN_DEPENDS
    roscpp
	 nav_msgs
	 quadrotor_msgs
	 tf
	 nodelet
    message_runtime
    geometry_msgs
  DEPENDS Eigen)

set(SOURCE_FILES
	   src/ControlCableVision.cpp
      src/vio_utils.cpp
)

set(HEADER_FILES
   include/control_cable_vision/ControlCableVision.h
   include/control_cable_vision/vio_utils.h
)

add_library(control_cable_vision_lib ${SOURCE_FILES})
target_link_libraries(control_cable_vision_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

add_library(nodelet_control_cable_vision src/nodelet_control_cable_vision.cpp)
add_dependencies(nodelet_control_cable_vision ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_control_cable_vision control_cable_vision_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})


#add_executable(ControlCableVision src/ControlCableVision.cpp)
#add_dependencies(ControlCableVision ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(ControlCableVision ${catkin_LIBRARIES})

#add_library(vio_utils src/vio_utils.cpp)
#target_link_libraries(vio_utils ${catkin_LIBRARIES})

#add_library(so3_control_nodelet src/so3_control_nodelet.cpp)
#add_dependencies(so3_control_nodelet ${catkin_EXPORTED_TARGETS})
#target_link_libraries(so3_control_nodelet SO3Control ${catkin_LIBRARIES})

#add_library(so3_trpy_control src/so3_trpy_control.cpp)
#add_dependencies(so3_trpy_control ${catkin_EXPORTED_TARGETS})
#target_link_libraries(so3_trpy_control SO3Control ${catkin_LIBRARIES})

#install(TARGETS
#	 ControlCableVision
#	 vio_utils
##    so3_control_nodelet
##    so3_trpy_control
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
#install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
