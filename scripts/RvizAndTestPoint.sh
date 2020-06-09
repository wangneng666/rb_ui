#!/bin/bash
roslaunch co605_dual_arm_gripper_moveit_config demo.launch &
roslaunch rubik_cube_solve solve.launch &
roslaunch grasp_place grasp.launch &
rosrun grasp_place test.py &
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
rosrun cubeParse cube &
wait
exit 0
