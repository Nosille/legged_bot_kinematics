#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Leg
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    origin= np.array([0.0, 0.073535, 0.0])
    lengths = np.array([0.0450503, 0.07703, 0.123, 0.000])
    offsets = np.array([math.pi/2, 0.0, -math.pi/2, -math.pi/2])

    leg = Leg("LM", origin, lengths, offsets)
    print(leg.name)

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





