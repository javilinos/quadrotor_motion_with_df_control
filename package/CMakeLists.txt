cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_motion_with_df_control)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)


# Directories definition
set(QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR
  src/source
)

set(QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR
  src/include
)

set(QUADROTOR_MOTION_WITH_DF_CONTROL_HEADER_FILES
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}/behavior_quadrotor_df_motion_control.h
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}/behavior_take_off_with_df.h
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}/behavior_land_with_df.h
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}/behavior_follow_path_with_df.h
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}/behavior_send_path.h
)

set(QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_FILES
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_quadrotor_df_motion_control.cpp
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_take_off_with_df.cpp
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_land_with_df.cpp
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_follow_path_with_df.cpp
        ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_send_path.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  ros_utils_lib
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${QUADROTOR_MOTION_WITH_DF_CONTROL_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  ros_utils_lib
  tf
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(BehaviorQuadrotorDFMotionControl ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_quadrotor_df_motion_control.cpp)
target_link_libraries(BehaviorQuadrotorDFMotionControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorQuadrotorDFMotionControl yaml-cpp)
add_executable(BehaviorTakeOffWithDF ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_take_off_with_df.cpp)
target_link_libraries(BehaviorTakeOffWithDF ${catkin_LIBRARIES})
target_link_libraries(BehaviorTakeOffWithDF yaml-cpp)
add_executable(BehaviorLandWithDF ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_land_with_df.cpp)
target_link_libraries(BehaviorLandWithDF ${catkin_LIBRARIES})
target_link_libraries(BehaviorLandWithDF yaml-cpp)
add_executable(BehaviorFollowPathWithDF ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_follow_path_with_df.cpp)
target_link_libraries(BehaviorFollowPathWithDF ${catkin_LIBRARIES})
target_link_libraries(BehaviorFollowPathWithDF yaml-cpp)
add_executable(BehaviorSendPath ${QUADROTOR_MOTION_WITH_DF_CONTROL_SOURCE_DIR}/behavior_send_path.cpp)
target_link_libraries(BehaviorSendPath ${catkin_LIBRARIES})
target_link_libraries(BehaviorSendPath yaml-cpp)


if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(BehaviorQuadrotorDFMotionControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorTakeOffWithDF ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorLandWithDF ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorFollowPathWithDF ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorSendPath ${catkin_EXPORTED_LIBRARIES})
endif()
