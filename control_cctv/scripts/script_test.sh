#!/bin/bash

ROBOT="dragonfly11"

echo "testing control_cctv functionality"

read -p "turn motors ON"
rosservice call /$ROBOT/mav_services/motors true

read -p "takeoff"
echo "Takeoff..."
rosservice call /$ROBOT/mav_services/takeoff

read -p "go to 2 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [2.0, 0.0, 1.0, 0.0]"

read -p "set desired payload to 2, 0, 0.3"
rostopic pub -1 /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose '{position: {x: 2.0, y: 0.0, z: 0.03}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

read -p "Press enter to switch controller"
rostopic pub -1 /$ROBOT/control_cctv/use_cctv_controller std_msgs/Bool 1

#read -p "Press enter to switch controller"
#rostopic pub -1 /$ROBOT/control_cctv/use_cctv_controller std_msgs/Bool 0

read -p "Press [Enter] to turn off motors"
rosservice call /$ROBOT/mav_services/motors 0
