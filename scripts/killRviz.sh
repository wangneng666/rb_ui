#!/bin/bash
rosnode kill $(rosnode list | grep 'rviz\|rosout') &
sleep 0.1s
rosnode kill /joint_state_dual_server &
sleep 0.1
rosnode kill /move_group &
sleep 0.3
rosnode kill /robot_state_publisher &
wait
exit 0
