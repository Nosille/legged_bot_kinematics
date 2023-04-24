#!/usr/bin/env python3

import rospy
from kinematics import Bot
# from legged_bot_kinematics import Leg
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

if __name__ == '__main__':
    
    roscpp_init('bot_node', [])

    legs = ["LF", "LM", "LR", "RF", "RM", "RR"]

    bot = Bot("jethexa", "body_link", legs)
    print(bot.name)

    x= input()
    


