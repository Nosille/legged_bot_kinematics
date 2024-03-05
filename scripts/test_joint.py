#!/usr/bin/env python3

import rospy
from leg_kinematics import Joint

if __name__ == '__main__':
    
    joint = Joint("joint1", -3.1416, 3.1416, 6.50)
    print(joint.name)
    print(joint.getMinAngle)
    print(joint.getMaxAngle)
    print(joint.getMaxRate)


