#!/bin/bash
roslaunch hsr_bringup co605_dual_arm_real.launch &
sleep 0.1
roslaunch gripper_bridge gripper_bridge_dual.launch &
sleep 0.1
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
sleep 0.1
rosrun grasp_place test.py &
sleep 0.1
roslaunch grasp_place grasp.launch &
sleep 0.1
rosrun gripper_bridge gripper.sh &
sleep 8
rosrun rubik_cube_solve set_robot_enable_true.sh &
sleep 0.1
wait
exit 0
