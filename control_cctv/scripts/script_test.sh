#!/bin/bash

ROBOT="dragonfly11"

echo "testing control_cctv functionality"

read -rsp "turn motors ON" -n1
rosservice call /$ROBOT/mav_services/motors true

read -rsp "takeoff" -n1
echo "Takeoff..."
rosservice call /$ROBOT/mav_services/takeoff

read -rsp "go to 0 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [0.0, 0.0, 1.0, 0.0]"

read -rsp "go to 4 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [4.0, 0.0, 1.0, 0.0]"

read -rsp "set desired payload to 4, 0, 0.5" -n1
rostopic pub /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose "position:
  x: 4.0
  y: 0.0
  z: 0.5
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"


read -rsp "Press enter to switch controller" -n1
rostopic pub --once /$ROBOT/control_cctv/use_cctv_controller std_msgs/Bool 1

read -rsp "Press [Enter] to land" -n1
rosservice call /$ROBOT/mav_services/land
sleep 1

read -rsp "Press [Enter] to turn off motors" -n1
rosservice call /$ROBOT/mav_services/motors 0
