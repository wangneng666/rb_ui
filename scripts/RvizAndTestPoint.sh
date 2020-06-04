#!/bin/bash
roslaunch rubik_cube_solve solve.launch speed:=1 &
sleep 0.5
roslaunch grasp_place grasp.launch &
sleep 0.5
rosrun grasp_place test.py &
sleep 0.5
roslaunch hsr_bringup publish_d435i_calibration_dual.launch &
sleep 0.5
rosrun cubeParse cube &
wait
exit 0
