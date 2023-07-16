#! /usr/bin/env python

"""
.. module::scanner
    :platform: Unix
    :synopsis: Python module for scanning the markers by moving the joints
.. moduleauthor:: Hussein Ahmed Fouad Hassan, S5165612@studenti.unige.it

ROS node for controlling the arm which holds the camera to scan the markers

Action server:

- action_scanner

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
