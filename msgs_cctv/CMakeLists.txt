cmake_minimum_required(VERSION 2.8.12)
project(msgs_cctv)

add_compile_options(-std=c++11)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  geometry_msgs 
  nav_msgs
  cmake_modules 
  rosconsole)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  PayloadCommand.msg
  )

generate_messages(
  DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs
  DEPENDS)

add_dependencies(${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
