cmake_minimum_required(VERSION 2.8.3)
project(copernic_gazebo)

set(CMAKE_BUILD_TYPE Release)

add_definitions("-std=c++11")

find_package(catkin)# REQUIRED COMPONENTS copernic_description copernic_controller copernic_sensors)

find_package(OpenCV REQUIRED)

catkin_package(
 # CATKIN_DEPENDS copernic_description copernic_controller copernic_sensors
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

