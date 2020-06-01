#!/bin/bash
roslaunch hsr_bringup co605_dual_arm_real.launch &
rosrun rubik_cube_solve set_robot_enable_true.sh &
roslaunch gripper_bridge gripper_bridge_dual.launch &
rosrun gripper_bridge gripper.sh &
roslaunch realsense2_camera rs_camera_right.launch &
roslaunch realsense2_camera rs_camera.launch &
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
roslaunch vision_bridge vision_bridge_yolo6d_dual.launch &
rosrun grasp_place test.py
sleep 0.1
roslaunch grasp_place grasp.launch &
wait
exit 0
