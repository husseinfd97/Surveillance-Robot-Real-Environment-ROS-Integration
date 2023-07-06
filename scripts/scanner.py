#! /usr/bin/env python

"""
.. module::scanner
    :platform: Unix
    :synopsis: Python module for the state machine
.. moduleauthor:: 

ROS node for controlling the camera arm of the proposed robot

Action server:

- action_scanner

"""

"""
This script is responsible for moving the arm of the robot in order to scan the room for markers.
It subscribes to the '/robot/joint1_position_controller/state' topic to get feedback on the current position of the arm and
publishes to the '/robot/joint1_position_controller/command' topic to control the movement of the arm.
The arm is moved from its initial position to -3.14 radians and then back to 1.5 radians before returning to its initial position at 0 radians.
"""

import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

feedback=0

def callback(msg):
    """
    Callback function to store the current position of the arm.
    """
    global feedback
    feedback = msg.process_value

def arm_move():
    """
    Function for moving the robot arm in a scanning motion.
    """
    print("Moving arm to scan around the room for markers")

    rospy.init_node('scanner', anonymous=True)
    pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/robot/joint1_position_controller/state", JointControllerState, callback)

    desired_positions = [-3.14, 1.5, 0.0]

    for position in desired_positions:
        while abs(feedback - position) > 0.01:
            pub.publish(position)
            rospy.sleep(0.1)

    print("Scanned the initial room, setting arm to initial position")
    rospy.signal_shutdown('marker_publisher')

if __name__ == '__main__':
    try:
        arm_move()
    except rospy.ROSInterruptException:
        pass
