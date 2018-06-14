cmake_minimum_required(VERSION 2.8.3)
project(payload_trajectory_cctv)

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
  msgs_cctv
  std_msgs
  cmake_modules)

find_package(Eigen REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
   ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    quadrotor_msgs
    geometry_msgs
    msgs_cctv
    std_msgs
  DEPENDS 
    Eigen)

 add_executable(simple_publisher src/simple_publisher.cpp)
 add_dependencies(simple_publisher ${catkin_EXPORTED_TARGETS})
 target_link_libraries(simple_publisher ${catkin_LIBRARIES} ${EIGEN_LIBS})

  add_executable(simple_payload_trajectory src/simple_payload_trajectory.cpp)
 add_dependencies(simple_payload_trajectory ${catkin_EXPORTED_TARGETS})
 target_link_libraries(simple_payload_trajectory ${catkin_LIBRARIES} ${EIGEN_LIBS})
