#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Bot


if __name__ == '__main__':
    
    legIds = ["LF", "LM", "LR", "RF", "RM", "RR"]
    origins = [np.array([ 0.09360, 0.050805, 0.0, math.pi*1/4]), 
               np.array([ 0.00000, 0.073535, 0.0, math.pi*1/2]), 
               np.array([-0.09360, 0.050805, 0.0, math.pi*3/4]), 
               np.array([ 0.09360,-0.050805, 0.0, math.pi*7/4]), 
               np.array([ 0.00000,-0.073535, 0.0, math.pi*3/2]), 
               np.array([-0.09360,-0.050805, 0.0, math.pi*5/4])] 
    legLengths = np.array([0.0450503, 0.07703, 0.123, 0.000])

    bot = Bot("jethexa", origins, legIds, legLengths)
    print(bot.name)

    print("setLegPosition")
    position = np.array([0.0, 0.140, -0.050])
    print(position)
    angles = bot.setLegPosition(1, position)
    print(angles)

    


