#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Leg
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    pose = np.array([0.0, 0.073535, 0.0, math.pi/2])
    lengths = np.array([0.0450503, 0.07703, 0.123, 0.000])
 
    leg = Leg("legLF", pose, lengths)
    print(leg.name)

    #inverse kinematics
    print("inverse kinematics")
    point = np.array([0.0, 0.140, -0.050])
    print(point)
    angles = leg.getAnglesFromPoint(point)
    print(angles)

    #forward kinematics
    # angles = {"coxa":0.0, "femur":1.244, "tibia":-2.842, "tarsus":0.000}
    print(angles)
    print("forward kinematics")
    point = leg.getPointFromAngles(angles)
    print(point)





