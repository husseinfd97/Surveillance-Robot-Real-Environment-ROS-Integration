#!/usr/bin/env python3
"""
.. module:: battery
  :platform: Unix 
  :synopsis: Python module for battery status control
.. moduleauthor:: Hussein Ahmed Fouad Hassan, S5165612@studenti.unige.it

The primary purpose of this module is to publish true or false according to status of the battery
to change the flag of robot battery.


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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


def battery_state_publisher():
    """
	Function which Publishes the battery state periodically to the 'battery_state' topic.

	"""
    
    
    pub = rospy.Publisher('battery_state', Bool, queue_size=10)
    rospy.init_node('battery_state_node', anonymous=True)
    rate = rospy.Rate(1.0 / 1)  # Publish rate of 1 Hz

    battery_state_msg = Bool()
    charged_duration = 300  
    recharge_duration = 50  

    while not rospy.is_shutdown():
        if charged_duration > 0:
            battery_state_msg.data = True
            rospy.loginfo('Battery state: %s', battery_state_msg.data)
            pub.publish(battery_state_msg)
            charged_duration -= 1
        elif recharge_duration > 0:
            battery_state_msg.data = False
            #cancel goals so the robot leaves everything and go to charge
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()
            rospy.loginfo('Battery state: %s', battery_state_msg.data)
            pub.publish(battery_state_msg)
            recharge_duration -= 1
        else:
            charged_duration = 30
            recharge_duration = 5

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        battery_state_publisher()
    except rospy.ROSInterruptException:
        pass
