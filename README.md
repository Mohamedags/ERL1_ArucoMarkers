# Experimental Robotics Laboratory  
## Assignment 1 – ROS 2 ArUco Recognition Using OpenCV

## Overview
A mobile robot is spawned in a simulated environment containing five ArUco markers with different IDs, arranged in a circular layout. The robot must rotate in place to detect and identify all markers.

Once all marker IDs have been detected, a ROS 2 node controls the robot to navigate toward the marker with the lowest ID, positioning it at the center of the camera image. The node then publishes a processed image on a custom topic, highlighting the detected marker with a circular overlay.

The robot repeats this procedure for each remaining marker in ascending ID order until all markers have been processed.
## Developed By
* Mohamed Aguenarous
* Lisa Mokrani

## Prerequisites
- ROS 2 (humble)
- gazebo, Rviz
- OpenCV
- OpenCV ArUco Module
- Markers were done with Blender and exported to gazebo

## Installation

1. Clone the required packages inside the `src` folder of ROS 2 workspace:
   ```bash
   git clone https://github.com/Mohamedags/ERL1_ArucoMarkers
   git clone https://github.com/carmineD8/ros2_aruco

2. Install the dependencies of the packages and build your workspace
   ```bash
   colcon build

3. Source your workspace.
   ```bash
   source install/setup.bash

## Running
   ```bash
   ros2 launch robot_urdf gazebo_launch.launch.py

