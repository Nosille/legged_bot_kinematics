#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Leg
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    
    origin= np.array([0.0, 0.073535, 0.0])
    lengths = np.array([0.0450503, 0.07703, 0.123, 0.000])
    offsets = np.array([math.pi/2, 0.0, -math.pi/2, 0.000])
    segmentMins = np.array([-2/3*math.pi, -2/3*math.pi, -2/3*math.pi, -2/3*math.pi])
    segmentMaxs = np.array([+2/3*math.pi, +2/3*math.pi, +2/3*math.pi, +2/3*math.pi])
    segmentRates = np.array([6.50, 6.50, 6.50, 6.50])

    print('Create Segment')
    leg = Leg("LM", origin, lengths, offsets, segmentMins, segmentMaxs, segmentRates)
    print(f'  name: {leg.name}')
    print(f'  segmentLengths: {leg.segmentLengths}')
    print(f'  segmentOffsets: {leg.segmentOffsets}')
    print(f'  segmentMins: {leg.segmentMinAngles}')
    print(f'  segmentMaxs: {leg.segmentMaxAngles}')
    print(f'  currentAngles: {leg.segmentCurrentAngles}')

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
    leg.setSegmentLengths(lengths_dict)
    print(f'  segmentLengths: {leg.segmentLengths}')

    offset_dict = {"coxa" : 0.02, "femur" : 0.04, "tibia" : 0.06, "tarsus" : 0.08}
    leg.setSegmentOffsets(offset_dict)
    print(f'  segmentOffsets: {leg.segmentOffsets}')

    min_dict = {"coxa" : -0.02, "femur" : -0.04, "tibia" : -0.06, "tarsus" : -0.08}
    leg.setSegmentMinAngles(min_dict)
    print(f'  segmentMins: {leg.segmentMinAngles}')
    
    max_dict = {"coxa" : 0.02, "femur" : 0.04, "tibia" : 0.06, "tarsus" : 0.08}
    leg.setSegmentMaxAngles(max_dict)
    print(f'  segmentMaxs: {leg.segmentMaxAngles}')

    currentAngle_dict = {"coxa" : 0.2, "femur" : 0.4, "tibia" : 0.6, "tarsus" : 0.8}
    leg.setSegmentCurrentAngles(currentAngle_dict)
    print(f'  currentAngles: {leg.segmentCurrentAngles}')






