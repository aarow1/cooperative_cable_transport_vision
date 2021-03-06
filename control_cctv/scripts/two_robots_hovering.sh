#!/bin/bash

ROBOT1="dragonfly11"
ROBOT2="dragonfly11"

echo "testing control_cctv functionality"

echo "First we'll get $ROBOT1 flying"
echo "$ROBOT1 Press the any key to: Turn motors ON"
read -rsn1
rosservice call /$ROBOT1/mav_services/motors true

echo "$ROBOT1 Press the any key to: Takeoff"
read -rsn1
rosservice call /$ROBOT/mav_services/takeoff

echo "$ROBOT1 Press the any key to: go to 2 0 1"
read -rsn1
rosservice call /$ROBOT/mav_services/goTo "goal: [2.0, 0.0, 1.0, 0.0]"

echo "$ROBOT1 Press the any key to: switch to the PL Hover tracker"
read -rsn1
rosservice call /dragonfly11/pl_trackers_manager/transition pl_trackers/PLHoverTracker

echo "$ROBOT1 Press the any key to: switch controllers"
read -rsn1
rosservice call /dragonfly11/control_cctv/use_cctv_controller "data: true"

#echo "Press the any key to set line tracker goal to current pos
#of 2, 0, 0.6"
#read -rsn1
#rostopic pub -1 /dragonfly11/pl_trackers_manager/line_tracker/goal quadrotor_msgs/LineTrackerGoal "{x: 2.0, y: 0.0, z: 0.3, yaw: 0.0, v_des: 0.1, a_des: 0.0, relative: false}"

#echo "Press the any key to transition to line tracker"
#read -rsn1
#rosservice call /dragonfly11/pl_trackers_manager/transition pl_trackers/PLLineTracker

#echo "Press the any key to go out"
#read -rsn1
#rostopic pub -1 /dragonfly11/pl_trackers_manager/line_tracker/goal quadrotor_msgs/LineTrackerGoal "{x: 6.0, y: 0.0, z: 0.5, yaw: 0.0, v_des: 0.5, a_des: 0.0, relative: false}"

#echo "Press the any key to right"
#read -rsn1
#rostopic pub -1 /dragonfly11/pl_trackers_manager/line_tracker/goal quadrotor_msgs/LineTrackerGoal "{x: 6.0, y: 0.5, z: 0.5, yaw: 0.0, v_des: 0.5, a_des: 0.0, relative: false}"

#echo "Press the any key to left"
#read -rsn1
#rostopic pub -1 /dragonfly11/pl_trackers_manager/line_tracker/goal quadrotor_msgs/LineTrackerGoal "{x: 6.0, y: -0.5, z: 0.5, yaw: 0.0, v_des: 0.5, a_des: 0.0, relative: false}"

#echo "Press the any key to come back"
#read -rsn1
#rostopic pub -1 /dragonfly11/pl_trackers_manager/line_tracker/goal quadrotor_msgs/LineTrackerGoal "{x: 1.5, y: 0.0, z: 0.2, yaw: 0.0, v_des: 0.25, a_des: 0.0, relative: false}

echo "Press the any key to: turn off motors"
read -rsn1
rosservice call /$ROBOT/mav_services/motors 0
