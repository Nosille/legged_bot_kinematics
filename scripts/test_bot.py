#!/usr/bin/env python3

import rospy
import math
import numpy as np
from leg_kinematics import Bot


if __name__ == '__main__':
    
    legIds = ["LF", "LM", "LR", "RF", "RM", "RR"]
    legOrigins = [np.array([ 0.09360, 0.050805, 0.0]), 
                  np.array([ 0.00000, 0.073535, 0.0]), 
                  np.array([-0.09360, 0.050805, 0.0]), 
                  np.array([ 0.09360,-0.050805, 0.0]), 
                  np.array([ 0.00000,-0.073535, 0.0]), 
                  np.array([-0.09360,-0.050805, 0.0])] 
    legLengths = [np.array([0.0450503, 0.07703, 0.123, 0.000]),
                  np.array([0.0450503, 0.07703, 0.123, 0.000]),
                  np.array([0.0450503, 0.07703, 0.123, 0.000]),
                  np.array([0.0450503, 0.07703, 0.123, 0.000]),
                  np.array([0.0450503, 0.07703, 0.123, 0.000]),
                  np.array([0.0450503, 0.07703, 0.123, 0.000])]
    jointOffsets = [np.array([+math.pi*1/4, 0.0, -math.pi/2, 0.0]), 
                    np.array([+math.pi*1/2, 0.0, -math.pi/2, 0.0]), 
                    np.array([+math.pi*3/4, 0.0, -math.pi/2, 0.0]), 
                    np.array([-math.pi*1/4, 0.0, -math.pi/2, 0.0]), 
                    np.array([-math.pi*1/2, 0.0, -math.pi/2, 0.0]), 
                    np.array([-math.pi*3/4, 0.0, -math.pi/2, 0.0])] 
    jointMins = [np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3]),
                 np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3]),
                 np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3]),
                 np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3]),
                 np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3]),
                 np.array([-math.pi*2/3, -math.pi*2/3, -math.pi*2/3, -math.pi*2/3])]
    jointMaxs = [np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3]),
                 np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3]),
                 np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3]),
                 np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3]),
                 np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3]),
                 np.array([+math.pi*2/3, +math.pi*2/3, +math.pi*2/3, +math.pi*2/3])]
    jointRates = [np.array([6.50, 6.50, 6.50, 6.50]),
                  np.array([6.50, 6.50, 6.50, 6.50]),
                  np.array([6.50, 6.50, 6.50, 6.50]),
                  np.array([6.50, 6.50, 6.50, 6.50]),
                  np.array([6.50, 6.50, 6.50, 6.50]),
                  np.array([6.50, 6.50, 6.50, 6.50])]                 
                    

    bot = Bot("jethexa", legIds, legOrigins, legLengths, jointOffsets, jointMins, jointMaxs, jointRates)
    print(bot.name)

    print("setLegPosition_LF")
    position = np.array([0.14060, 0.097803, -0.050])
    print(position)
    angles = bot.setLegPosition(0, position)
    print(angles)

    print("setLegPosition_LM")
    position = np.array([0.0, 0.140, -0.050])
    print(position)
    angles = bot.setLegPosition(1, position)
    print(angles)

    print("setLegPosition_LR")
    position = np.array([-0.14060, 0.097803, -0.050])
    print(position)
    angles = bot.setLegPosition(2, position)
    print(angles)

    print("setLegPosition_RF")
    position = np.array([0.14060, -0.097803, -0.050])
    print(position)
    angles = bot.setLegPosition(3, position)
    print(angles)  

    print("setLegPosition_RM")
    position = np.array([0.0, -0.140, -0.050])
    print(position)
    angles = bot.setLegPosition(4, position)
    print(angles)

    print("setLegPosition_RR")
    position = np.array([-0.14060, -0.097803, -0.050])
    print(position)
    angles = bot.setLegPosition(5, position)
    print(angles)          
   


#   in local coordinates origin to foot is
#   x = 0.066465 y = 0 z = -0.050
#   at 45 deg
#   x = 0.0469978522 y = 0.0469978522 z = -0.050