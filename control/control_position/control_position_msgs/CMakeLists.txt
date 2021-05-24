cmake_minimum_required(VERSION 2.8.3)
project(control_position_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
  FILES
  ControllerReference.msg
  ControllerMode.msg
  ControllerFeedback.msg
  AccelerationYawrate.msg
  TeleopVisualisation.msg
  )
  
add_service_files(
  FILES
  AutoThrust.srv
  EmergencySIP.srv
  )
  
generate_messages(  DEPENDENCIES  
  std_msgs
  geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_position_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


# install(TARGETS control_position_msgs
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
