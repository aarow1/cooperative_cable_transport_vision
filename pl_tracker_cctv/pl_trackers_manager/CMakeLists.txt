cmake_minimum_required(VERSION 2.8.3)
project(pl_trackers_manager)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib quadrotor_msgs nav_msgs msgs_cctv)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv
  FILES
  Transition.srv
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pl_trackers_manager
  CATKIN_DEPENDS roscpp nodelet pluginlib quadrotor_msgs nav_msgs msgs_cctv
  DEPENDS
)

add_library(pl_trackers_manager src/pl_trackers_manager.cpp)
add_dependencies(pl_trackers_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pl_trackers_manager ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
