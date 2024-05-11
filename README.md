# 3D Arm Kinematics Analysis and Control

## Overview
This repository contains the implementation of a 3D kinematic analysis and control system for an anthropomorphic robotic arm with three joints. The project utilizes Denavit-Hartenberg parameters to achieve precise forward and inverse kinematics calculations, showcasing an understanding of robotics theory in practical applications.

## Prerequisites
- ROS (Robot Operating System)
- Gazebo for simulation
- RViz for visualization

Ensure that ROS and Gazebo are correctly installed on your system.

## Installation
Clone the repository and build the workspace:
```bash
cd ~
git clone https://github.com/MiguelSolisSegura/arm_kinematics.git
cd arm_kinematics
catkin_make
```

## Running the Simulation
To launch the simulation environment and the RViz visualization, use the following commands:

### Start the Gazebo Simulation
```bash
source devel/setup.bash
roslaunch antropomorphic_arm_gazebo main.launch
```

### Launch RViz
```bash
rosrun rviz rviz -d src/planar_3dof_kinematics/antropomorphic_arm_description/rviz/antropomorphic_3dof.rviz
```

### Control the Robot Arm
To move the robot arm and see the kinematics in action, open a new terminal and run:
```bash
roslaunch antropomorphic_arm_control antropomorphic_arm_sim_rqt.launch
```

## Testing Forward and Inverse Kinematics
Ensure that all components are running and then execute the commands to test the kinematics calculations.

### Forward Kinematics
```bash
rosrun antropomorphic_project fk_antropomorphic_arm.py
```
Enter the joint angles as prompted to see the resulting end-effector position and orientation.

### Inverse Kinematics
```bash
rosrun antropomorphic_project ik_antropomorphic_arm.py
```
Input the desired position of the end-effector to receive the joint angles that achieve that position.

