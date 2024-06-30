#!/usr/bin/env python3

import rospy
from leg_kinematics import Joint

if __name__ == '__main__':
    
    print('Create Joint')
    joint = Joint("joint1", 0.123, 3.1416/2.0, -3.1416, 3.1416, 6.50)
    print(f'  name: {joint.name}')
    print(f'  length: {joint.length}')
    print(f'  offset: {joint.offset}')
    print(f'  minAngle: {joint.minAngle}')
    print(f'  maxAngle: {joint.maxAngle}')
    print(f'  maxRate: {joint.maxRate}')
    print(f'  currentAngle : {joint.currentAngle}')

    print('')
    print('Change currentAngle')
    joint.setCurrentAngle(3.1416/4.0)
    print(f'  currentAngle : {joint.currentAngle}')




