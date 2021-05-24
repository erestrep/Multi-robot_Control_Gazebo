cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_controller)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer -Wall -Wextra " ) # -Weffc++ for profiler and some warning 
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -Wall -Wextra " ) # for profiler and some warning -Weffc++
add_definitions("-std=c++11")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rospy
  dynamic_reconfigure
  control_position_msgs
  #~ mapping_stack
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

generate_dynamic_reconfigure_options(
  cfg/MobileRobotController.cfg
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/mobile_robot_controller.cpp src/controller_node.cpp) # src/MPC_mobile_robot.cpp)
add_executable(delay_filters src/delayComm.cpp src/delay_node.cpp) # src/MPC_mobile_robot.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(delay_filters ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
