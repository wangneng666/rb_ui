#!/bin/bash
roslaunch hsr_bringup co605_dual_arm_real.launch &
sleep 2
roslaunch realsense2_camera rs_camera_right.launch &
roslaunch realsense2_camera rs_camera.launch &
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
sleep 0.5
roslaunch gripper_bridge gripper_bridge_dual.launch &
sleep 3
wait
exit 0
