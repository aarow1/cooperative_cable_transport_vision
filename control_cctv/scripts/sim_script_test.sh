#!/bin/bash

ROBOT="dragonfly11"

echo "testing control_cctv functionality"

read -p "turn motors ON"
rosservice call /$ROBOT/mav_services/motors true

read -p "takeoff"
echo "Takeoff..."
rosservice call /$ROBOT/mav_services/takeoff

read -p "go to 0.1 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [0.1, 0.0, 1.0, 0.0]"

read -p "go to -0.1 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [-0.1, 0.0, 1.0, 0.0]"

read -p "go to 0 0 1"
rosservice call /$ROBOT/mav_services/goTo "goal: [0.0, 0.0, 1.0, 0.0]"

read -p "set desired payload to 0.5, 0, 0.0"
rostopic pub -1 /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose '{position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

read -p "set desired payload to -0.5, 0, 0.0"
rostopic pub -1 /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose '{position: {x: -0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

read -p "set desired payload to 0, 0.5, 0.0"
rostopic pub -1 /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose '{position: {x: 0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

read -p "set desired payload to 0, -0.5, 0.0"
rostopic pub -1 /dragonfly11/control_cctv/payload_cmd geometry_msgs/Pose '{position: {x: 0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

read -p "Press [Enter] to turn off motors"
rosservice call /$ROBOT/mav_services/motors 0
