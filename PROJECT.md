# MicroUWB-Nav Simulation

Sub-50g indoor quadcopter simulation with UWB-based positioning.
Proof-of-concept for the physical MicroUWB-Nav drone (nRF52840 + DW3000).

## Stack
ROS2 Humble | Gazebo Fortress | Sionna | Python 3.10 | Ubuntu 22.04

## Package Map
- microuwb_msgs → custom messages
- microuwb_description → drone URDF/SDF
- microuwb_gazebo → room world, anchors, launch
- microuwb_uwb → range sim, trilateration, EKF
- microuwb_control → attitude + position PID
- microuwb_navigation → waypoints, wall avoidance, RTH
- microuwb_bringup → integration launch files
- microuwb_rviz → visualization configs

## Build
colcon build --symlink-install
source install/setup.bash

## Branch Strategy
main ← stable integration
feature/<component>-<short-desc>