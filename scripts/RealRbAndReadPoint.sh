#!/bin/bash
rosrun rubik_cube_solve set_robot_enable_true.sh &
sleep 2
roslaunch vision_bridge vision_bridge_yolo6d_dual.launch &
sleep 0.5
roslaunch grasp_place grasp.launch &
sleep 0.5
roslaunch rubik_cube_solve solve.launch &
sleep 0.5
rosrun cubeParse cube &
wait
exit 0
