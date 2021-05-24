cmake_minimum_required(VERSION 2.8.3)
project(ground_truth_manager)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

find_package(gazebo REQUIRED)

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS sensor_msgs nav_msgs gazebo_msgs
)



add_executable(ground_truth_manager src/ground_truth_manager.cpp)
add_dependencies(ground_truth_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_truth_manager ${catkin_LIBRARIES})
