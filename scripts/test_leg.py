#!/usr/bin/env python3

import rospy
import numpy as np
from kinematics import Leg
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    pose = np.array([-2.0, -1.0, 0.0, -0.785398])
    lengths = np.array([0.101, 0.102, 0.103, 0.104])
 
    leg = Leg("legLF", pose, lengths)
    print(leg.name)

    #inverse kinematics
    print("inverse kinematics")
    point = np.array([-2.14424978336, -1.14424978336, -0.002])
    print(point)
    angles = leg.getAnglesFromPoint(point)
    print(angles)

    #forward kinematics
    print("forward kinematics")
    point = leg.getPointFromAngles(angles)
    print(point)


