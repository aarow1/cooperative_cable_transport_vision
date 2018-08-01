cmake_minimum_required(VERSION 2.8.3)
project(pl_trackers)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs tf pl_trackers_manager)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs tf pl_trackers_manager
  DEPENDS EIGEN3)

add_library(pl_initial_conditions src/pl_initial_conditions.cpp)
target_link_libraries(pl_initial_conditions ${catkin_LIBRARIES})

add_library(pl_line_tracker_min_jerk src/pl_line_tracker_min_jerk.cpp)
add_dependencies(pl_line_tracker_min_jerk ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pl_line_tracker_min_jerk pl_initial_conditions ${catkin_LIBRARIES})

add_library(pl_null_tracker src/pl_null_tracker.cpp)
add_dependencies(pl_null_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pl_null_tracker ${catkin_LIBRARIES})

add_library(pl_hover_tracker src/pl_hover_tracker.cpp)
add_dependencies(pl_hover_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pl_hover_tracker ${catkin_LIBRARIES})

install(TARGETS
    pl_initial_conditions
    pl_line_tracker_min_jerk
    pl_null_tracker
    pl_hover_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
