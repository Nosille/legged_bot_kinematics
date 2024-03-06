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

    print('Create Joint')
    leg = Leg("LM", origin, lengths, offsets, jointMins, jointMaxs, jointRates)
    print(f'  name: {leg.name}')
    print(f'  jointLengths: {leg.jointLengths}')
    print(f'  jointOffsets: {leg.jointOffsets}')
    print(f'  jointMins: {leg.jointMinAngles}')
    print(f'  jointMaxs: {leg.jointMaxAngles}')
    print(f'  currentAngles: {leg.jointCurrentAngles}')

    #inverse kinematics
    point = np.array([0.0, 0.140, -0.050])
    print("inverse kinematics")
    print(f'  point: {point}')
    angles = leg.getAnglesFromPoint(point)
    print(f'  angles: {angles}')

    #forward kinematics
    # angles = {"coxa":0.0, "femur":1.244, "tibia":-2.842, "tarsus":0.000}
    print("forward kinematics")
    print(f'  angles: {angles}')

    point = leg.getPointFromAngles(angles)
    print(f'  point: {point}')

    print('')
    print('Change currentValues')

    lengths_dict = {"coxa" : 0.05, "femur" : 0.06, "tibia" : 0.07, "tarsus" : 0.08}
    leg.setJointLengths(lengths_dict)
    print(f'  jointLengths: {leg.jointLengths}')

    offset_dict = {"coxa" : 0.02, "femur" : 0.04, "tibia" : 0.06, "tarsus" : 0.08}
    leg.setJointOffsets(offset_dict)
    print(f'  jointOffsets: {leg.jointOffsets}')

    min_dict = {"coxa" : -0.02, "femur" : -0.04, "tibia" : -0.06, "tarsus" : -0.08}
    leg.setJointMinAngles(min_dict)
    print(f'  jointMins: {leg.jointMinAngles}')
    
    max_dict = {"coxa" : 0.02, "femur" : 0.04, "tibia" : 0.06, "tarsus" : 0.08}
    leg.setJointMaxAngles(max_dict)
    print(f'  jointMaxs: {leg.jointMaxAngles}')

    currentAngle_dict = {"coxa" : 0.2, "femur" : 0.4, "tibia" : 0.6, "tarsus" : 0.8}
    leg.setJointCurrentAngles(currentAngle_dict)
    print(f'  currentAngles: {leg.jointCurrentAngles}')






