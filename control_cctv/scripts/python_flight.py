#!/usr/bin/env python

from mav_manager.srv import *
from std_srvs.srv import *
from pl_trackers_manager.srv import *
import rospy

robot1 = "dragonfly11"
robot2 = "dragonfly2"

estop_client1            = rospy.ServiceProxy(robot1 + '/mav_services/estop', Trigger)
motors_client1           = rospy.ServiceProxy(robot1 + '/mav_services/motors', SetBool)
takeoff_client1          = rospy.ServiceProxy(robot1 + '/mav_services/takeoff', Trigger)
goto_client1             = rospy.ServiceProxy(robot1 + '/mav_services/goTo', Vec4)
gotorelative_client1     = rospy.ServiceProxy(robot1 + '/mav_services/goToRelative', Vec4)
gohome_client1           = rospy.ServiceProxy(robot1 + '/mav_services/goHome', Trigger)
land_client1             = rospy.ServiceProxy(robot1 + '/mav_services/land', Trigger)
pltm_transition_client1  = rospy.ServiceProxy(robot1 + '/pl_trackers_manager/transition', Transition)
use_cctv_control_client1 = rospy.ServiceProxy(robot1 + '/control_cctv/use_cctv_controller', SetBool)

estop_client2            = rospy.ServiceProxy(robot2 + '/mav_services/estop', Trigger)
motors_client2           = rospy.ServiceProxy(robot2 + '/mav_services/motors', SetBool)
takeoff_client2          = rospy.ServiceProxy(robot2 + '/mav_services/takeoff', Trigger)
goto_client2             = rospy.ServiceProxy(robot2 + '/mav_services/goTo', Vec4)
gotorelative_client2     = rospy.ServiceProxy(robot2 + '/mav_services/goToRelative', Vec4)
gohome_client2           = rospy.ServiceProxy(robot2 + '/mav_services/goHome', Trigger)
land_client2             = rospy.ServiceProxy(robot2 + '/mav_services/land', Trigger)
pltm_transition_client2  = rospy.ServiceProxy(robot2 + '/pl_trackers_manager/transition', Transition)
use_cctv_control_client2 = rospy.ServiceProxy(robot2 + '/control_cctv/use_cctv_controller', SetBool)

pos1 = [5.5, 0, 1, 0]
pos2 = [4.5, 0, 1, 0]

land1 = [5.8, 0, -0.2, 0]
land2 = [4.2, 0, -0.2, 0]

flow = [
    ("1 turn motors on",          motors_client1,  1),
    ("2 turn motors on",          motors_client2,  1),

    ("1 takeoff",                 takeoff_client1, ),
    ("2 takeoff",                 takeoff_client2, ),

    ("1 go to " + str(pos1),      goto_client1,    pos1),
    ("2 go to " + str(pos2),      goto_client2,    pos2),

    ("1 set cctv hover",          pltm_transition_client1,    'pl_trackers/PLHoverTracker'),
    ("2 set cctv hover",          pltm_transition_client2,    'pl_trackers/PLHoverTracker'),

    ("1 switch to cctv control",  use_cctv_control_client1, 1),
    ("2 switch to cctv control",  use_cctv_control_client2, 1),

    ("1 switch to so3 control",   use_cctv_control_client1, 0),
    ("2 switch to so3 control",   use_cctv_control_client2, 0),

    ("1 go to " + str(land1),     goto_client1,    land1),
    ("2 go to " + str(land2),     goto_client2,    land2),

    ("1 turn motors off",         motors_client1,  0),
    ("2 turn motors off",         motors_client2,  0),
]

if __name__ == "__main__":
    ind = 0
    while ind < len(flow):
        print("\nPress [Enter] to: " + flow[ind][0] + "   or Press [e] to: estop")
        ans = raw_input()
        if ans.lower() == "e":
            estop_client1()
            estop_client2()
            break

        elif ans.lower() == "h":
            use_cctv_control_client1(0)
            use_cctv_control_client2(0)

        elif ans.lower() == "l":
            goto_client1(land1)
            goto_client2(land2)

        # elif ans == "m":
        #     motors_client(1)
        # elif ans == "M":
        #     motors_client(0)
        # elif ans == "t":
        #     takeoff_client()
        #
        # elif ans == "x":
        #     gotorelative_client([ 0.5, 0, 0, 0])
        # elif ans == "X":
        #     gotorelative_client([-0.5, 0, 0, 0])
        #
        # elif ans == "y":
        #     gotorelative_client([0,  0.5, 0, 0])
        # elif ans == "Y":
        #     gotorelative_client([0, -0.5, 0, 0])
        #
        # elif ans == "z":
        #     gotorelative_client([0, 0,  0.5, 0])
        # elif ans == "Z":
        #     gotorelative_client([0, 0, -0.5, 0])

        else:
            tup = flow[ind]
            if len(tup) == 2:
                print(tup[1]())
            elif len(tup) == 3:
                print(tup[1](tup[2]))
            ind = ind + 1
