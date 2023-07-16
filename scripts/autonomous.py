#! /usr/bin/env python
"""

.. module:: autonomous
    :platform: Unix
    :synopsis: Python code for autonomous robot drive 
.. moduleauthor:: Hussein Hassan  S5165612@studenti.unige.it

Service:
    /robot_coordinates which make the robot move to the set x and y coordinates 
Actions:
    /move_base the goal of entered x and y to check if it arrived to it or not

This node is used for the user choice number one which is resposible for  auronomous robot drive.

"""

import rospy
from assignment2.srv import robot_coordinates	
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *


def handler(req):
    """
    This function is used for handeling the services as it waits for the user enter the x and y coordinates
    The user message is passed to the service
    ``robot_coordinates``,
    advertised by :mod:`user_control`. 
    and then using actions to insure if the robot reached the right goal or not.

    arg: 
        req(int):the services element for recieving x and y coordinates
    """
    #save axis positions entered by the user 
    x = req.x
    y = req.y
    
    #Starting the action 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    #wait for  server 
    client.wait_for_server()
    

    #create and set the goal which entered by the user and check if it's reachable or not and if not cancel the goal
    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    #send the goal to client
    client.send_goal(goal)
    wait = client.wait_for_result(timeout=rospy.Duration(50.0))
    if not wait:
        #if it's in this loop means the target not reachable(out of range)
    	print("abort the mission!!")
    	client.cancel_goal()
    	return -1
    #out the loop the target reachable
    return 1


   

#main
if __name__=="__main__":
    
    print(" (autonomous node )")
    rospy.init_node('coordinate_autonomous')
    s = rospy.Service('robot_coordinates' ,robot_coordinates ,handler)
    print("service ready")
    rospy.spin()
