#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Leg
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    origin= np.array([0.0, 0.073535, 0.0])
    lengths = np.array([0.0450503, 0.07703, 0.123, 0.000])
    offsets = np.array([math.pi/4, 0.0, -math.pi/2, -math.pi/2])
    jointMins = np.array([-2/3*math.pi, -2/3*math.pi, -2/3*math.pi, -2/3*math.pi])
    jointMaxs = np.array([+2/3*math.pi, +2/3*math.pi, +2/3*math.pi, +2/3*math.pi])
    jointRates = np.array([6.50, 6.50, 6.50, 6.50])

    leg = Leg("LM", origin, lengths, offsets, jointMins, jointMaxs, jointRates)
    print(leg.name)
    # print(leg.origin)
    print(leg.lengths)
    print(leg.offsets)

    #inverse kinematics
    point = np.array([0.0, 0.140, -0.050])
    print("inverse kinematics")
    print(point)
    angles = leg.getAnglesFromPoint(point)
    print(angles)

    #forward kinematics
    # angles = {"coxa":0.0, "femur":1.244, "tibia":-2.842, "tarsus":0.000}
    print("forward kinematics")
    print(angles)
    point = leg.getPointFromAngles(angles)
    print(point)





