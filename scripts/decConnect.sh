#!/bin/bash
roslaunch hsr_bringup co605_dual_arm_real.launch &
roslaunch gripper_bridge gripper_bridge_dual.launch &
roslaunch realsense2_camera rs_camera_right.launch &
roslaunch realsense2_camera rs_camera.launch &
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
roslaunch vision_bridge vision_bridge_yolo6d_dual.launch &
roslaunch grasp_place grasp.launch &
roslaunch rubik_cube_solve solve.launch &
rosrun cubeParse cube &
wait
exit 0
