cmake_minimum_required(VERSION 2.8.3)
project(control_cctv)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nav_msgs
	quadrotor_msgs
   geometry_msgs
   msgs_cctv
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
      src/ControlCCTV.cpp
      src/SO3Control.cpp
      src/vio_utils.cpp
)

set(HEADER_FILES
   include/control_cctv/ControlCCTV.h
   include/control_cctv/SO3Control.h
   include/control_cctv/vio_utils.h
)

add_library(control_cctv_lib ${SOURCE_FILES})
target_link_libraries(control_cctv_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

add_library(nodelet_control_cctv src/NodeletControlCCTV.cpp)
add_dependencies(nodelet_control_cctv ${catkin_EXPORTED_TARGETS})
target_link_libraries(nodelet_control_cctv control_cctv_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

install(TARGETS nodelet_control_cctv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
