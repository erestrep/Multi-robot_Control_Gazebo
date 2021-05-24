# Gazebo-Control
Repository for gazebo simulation and control of mobile robots

## Gazebo simulation stack for Pioneer P3DX
## Requirement
- Gazebo 9 / ROS Melodic

### Dependency packages
- ros-melodic-rotors-*
- ros-kinetic-joint-state-controller
- ros-kinetic-ros-controllers
- ros-kinetic-ros-control
- ros-kinetic-velocity-controllers
- ros-kinetic-view-controller-msgs
- ros-kinetic-robot-state-publisher
- ros-kinetic-diff-drive-controller
- ros-kinetic-effort-controllers

## Organisation
common:
- copernic_gazebo : contains launch files for gazebo simulation
- ground_truth_manager : node publishing odometry topic of all gazebo objects

control:
- control_position: msg definition for the control
- mobile_robot_controller: scripts for the control of a ground mobile robot

hardware:
- p3dx-sim: gazebo description files of robots and sensors
