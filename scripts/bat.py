#!/usr/bin/env python3
"""
.. module:: batterynode
    :platform: Unix
    :synopsis: Python module for publishing the battery state to the topic (batterylevel)

.. moduleauthor:: 
"""

import roslib
import random
import math
import time
import rospy
import rospkg
import smach
import smach_ros
from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

def battery_state_publisher():
    pub = rospy.Publisher('battery_state', Bool, queue_size=10)
    rospy.init_node('battery_state_node', anonymous=True)
    rate = rospy.Rate(1.0 / 1)  # Publish rate of 1 Hz

    battery_state_msg = Bool()
    charged_duration = 15  # Duration in seconds for being fully charged
    recharge_duration = 5  # Duration in seconds for needing to recharge

    while not rospy.is_shutdown():
        if charged_duration > 0:
            battery_state_msg.data = True
            rospy.loginfo('Battery state: %s', battery_state_msg.data)
            pub.publish(battery_state_msg)
            charged_duration -= 1
        elif recharge_duration > 0:
            battery_state_msg.data = False
            rospy.loginfo('Battery state: %s', battery_state_msg.data)
            pub.publish(battery_state_msg)
            recharge_duration -= 1
        else:
            charged_duration = 15
            recharge_duration = 5

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        battery_state_publisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        battery_state_publisher()
    except rospy.ROSInterruptException:
        pass






if __name__ == '__main__':
    try:
        battery_state_msg = Bool()  # Declare battery_state_msg with a default value
        battery_state_publisher()
    except rospy.ROSInterruptException:
        pass
