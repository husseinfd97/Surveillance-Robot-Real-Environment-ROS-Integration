#!/usr/bin/env python
"""
.. module:: finite_state_machine
  :platform: Unix 
  :synopsis: Python module for the node state machine control
.. moduleauthor:: Hussein Ahmed Fouad Hassan, S5165612@studenti.unige.it

The primary purpose of this module is to execute the state machine for the robot.
It effectively manages state transitions and ensures a clear and organized implementation 
of the robot's state machine.


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
from std_msgs.msg import Float64
from assignment2.srv import robot_coordinates
from geometry_msgs.msg import Twist
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
import datetime
import helper

# Get the map direction to be uploaded  
path = dirname(realpath(__file__))
new_map = path + "/../maps/new_map.owl"

 
# Set the initial values of the status flages
map_Uploaded_flag=0
battery_charged_flag=1
urgent_room_flag=0
planning_finished_flag=1
coordinates=[]


       
def move_in_the_map(desired):
    """
    This function moves the real robot to the specified location in the map.

    Args:
        desired (str): The name of the location to move the robot to.

    Returns:
        rt: The result of the cordinates_srv service call, indicating whether the target was reached successfully.
    """
    tx=0
    cordinates_srv = rospy.ServiceProxy('robot_coordinates', robot_coordinates)
    x=coordinates[desired]['X']
    y=coordinates[desired]['Y']
    print(cordinates_srv)
    rt=cordinates_srv(x, y)
    print(rt)
    if rt.return_ == 1:
            print("Target reached successfully!")
            tx=1
    else:
            print("Target not reached try again!")
            move_in_the_map(desired)
            tx=0
    return tx


def round_in_the_room():
    """
    Function for scanning the environment by rotating the robot in place.

    This function rotates the robot in place by publishing a Twist message with a positive angular.z value to the 'cmd_vel' topic.
    It does this for 10 iterations at a rate of 10Hz, and then sleeps for 5 seconds.

    Args:
        None

    Returns:
        None
    """
    print("Scanning the room to map it...")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist_msg = Twist()
    twist_msg.angular.z = 1.0  # Rotate with positive angular.z value
    rate = rospy.Rate(10)
    iterations = 10
    while iterations > 0:
        pub.publish(twist_msg)
        rate.sleep()
        iterations -= 1
    rospy.sleep(5)

def set_map_coordinates():
    """
    Initially, a list of room names is generated. For each room in the list, the ArmorClient
    is used to query and retrieve the respective X and Y coordinates. These values are stored
    in variables 'X' and 'Y' from the map after it's loaded, respectively. Subsequently, the function
    populates the 'coordinates'dictionary with the room name as the key and a nested dictionary containing 
    the X and Y coordinates as the value.


   
    """
    global coordinates
    coordinates = {}  # Initialize coordinates as an empty dictionary

    client = ArmorClient("example", "ontoRef")
    list_of_rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    for i in list_of_rooms:
        req = client.call('QUERY', 'DATAPROP', 'IND', ['X-coordinates', i])
        X = helper.get_time(req.queried_objects)
        print(X)
        if X is not None:
            X = float(X)
        else:
            # Handle the case when X is None (provide a default value or take appropriate action)
            X = 0.0  # Example: Set X to 0.0
        req = client.call('QUERY', 'DATAPROP', 'IND', ['Y-coordinates', i])
        Y = helper.get_time(req.queried_objects)
        if Y is not None:
            Y = float(Y)
        else:
            # Handle the case when Y is None (provide a default value or take appropriate action)
            Y = 0.0  # Example: Set Y to 0.0
        coordinates[i] = {'X': X, 'Y': Y}
        print(coordinates[i])


def check_nearby_urgent_room():
    """
    Check for nearby urgent rooms, update the global `urgent_room_flag`, and return the urgent room if it exists.

    Returns:
        str: The urgent room nearby, according to the urgency of each robot. Returns '0' if no urgent room is found.
    """
    global urgent_room_flag
    urgent_room_flag = 0  # Initially, assume there's no urgent room
    least_room = None

    client = ArmorClient("example", "ontoRef")
    client.call('REASON', '', '', [''])

    # Query for urgent rooms
    urgent_rooms_query = client.call('QUERY', 'IND', 'CLASS', ['URGENT'])
    #print(helper.list_Locations(urgent_rooms_query.queried_objects))
    urgent_rooms_query = helper.list_Locations(urgent_rooms_query.queried_objects)
    client.call('REASON', '', '', [''])

    # Query current robot location
    current_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    current_location = helper.clean_list(current_location_query.queried_objects)

    if current_location == 'C1':
        room1_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R1'])
        room2_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R2'])
        room1_visit_time = float(helper.get_time(room1_visit_time_query.queried_objects))
        room2_visit_time = float(helper.get_time(room2_visit_time_query.queried_objects))
        least_room = helper.get_least_visit_time_room('R1', room1_visit_time, 'R2', room2_visit_time)
    elif current_location == 'C2':
        room3_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R3'])
        room4_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R4'])
        room3_visit_time = float(helper.get_time(room3_visit_time_query.queried_objects))
        room4_visit_time = float(helper.get_time(room4_visit_time_query.queried_objects))
        least_room = helper.get_least_visit_time_room('R3', room3_visit_time, 'R4', room4_visit_time)

    room = helper.same_elements_bt_lists([least_room], urgent_rooms_query)

    if room is not None:
        if "R1" in room:
            urgent_room_flag = 1
            return 'R1'
        elif "R2" in room:
            urgent_room_flag = 1
            return 'R2'
        elif "R3" in room:
            urgent_room_flag = 1
            return 'R3'
        elif "R4" in room:
            urgent_room_flag = 1
            return 'R4'

    return '0'  # No nearby urgent room found

def navigate_to(target_location):
    """
    Navigate the robot to the target location and check the best path to do that .

    Args:
        target_location (str): The location to which the robot should move.
    """
    client = ArmorClient("example", "ontoRef")
    client.call('REASON', '', '', [''])
    

    # Query initial robot location
    initial_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    current_location = helper.clean_list(initial_location_query.queried_objects)
    print('From', current_location, 'to:', target_location)

    if target_location == current_location:
        # If the target location is the same as the current location, directly move to the target location
        update_location_property(client, 'Robot1', target_location, current_location)
        update_now_property(client, 'Robot1')
        check_and_update_visitedat_property(client, target_location)
    else:
        reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
        reachable_locations = helper.list_Locations(reachable_locations_query.queried_objects)
        if target_location in reachable_locations:
            #print('check1')
            update_location_property(client, 'Robot1', target_location, current_location)
            #print('check2')
            update_now_property(client, 'Robot1')
            check_and_update_visitedat_property(client, target_location)
        else:
            print(reachable_locations)
            #print('check3')
            print(target_location)
            potential_path = generate_path_to_target(client, current_location, target_location, reachable_locations)
            print(potential_path)
            intermediate_location = potential_path[0]
            current_location = potential_path[2]
            #print('check4')
            print(target_location)
            print(intermediate_location)
            print(current_location)
            update_location_property(client, 'Robot1', intermediate_location, current_location)
            #print('check5')
            hena = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
            hena = helper.clean_list(hena.queried_objects)
            print('Robot here at:', hena)
            update_now_property(client, 'Robot1')
            check_and_update_visitedat_property(client, intermediate_location)
            current_location = intermediate_location

            reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
            reachable_locations = helper.list_Locations(reachable_locations_query.queried_objects)

            if target_location in reachable_locations:
                #print('check6')
                update_location_property(client, 'Robot1', target_location, current_location)
                #print('check7')
                update_now_property(client, 'Robot1')
                check_and_update_visitedat_property(client, target_location)

    final_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    print('Robot finally isIn', helper.clean_list(final_location_query.queried_objects))


def update_location_property(client, robot, new_location, old_location):
    """
    Update the location property of the robot.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        robot (str): The name of the robot.
        new_location (str): The new location to update.
        old_location (str): The old location to replace.
    """
    print(new_location)

    rt=move_in_the_map(new_location)
    if rt == 1:
        client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', robot, new_location, old_location])
        client.call('REASON','','',[''])

def update_now_property(client, robot):
    """
    Update the current time property of the robot.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        robot (str): The name of the robot.
    """
    current_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['now', robot])
    current_time = helper.get_time(current_time_query.queried_objects)
    client.call('REPLACE', 'DATAPROP', 'IND',
                ['now', robot, 'Long', str(math.floor(time.time())), current_time])
    client.call('REASON', '', '', [''])


def check_and_update_visitedat_property(client, location):
    """
    Check and update the visitedAt property of the given location.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        location (str): The location to check and update.
    """
    location_class_query = client.call('QUERY', 'CLASS', 'IND', [location, 'true'])
    
    if location_class_query.queried_objects == ['URGENT'] or location_class_query.queried_objects == ['ROOM']:
        visited_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', location])
        visited_time = helper.get_time(visited_time_query.queried_objects)
        client.call('REPLACE', 'DATAPROP', 'IND',
                    ['visitedAt', location, 'Long', str(math.floor(time.time())), visited_time])
        client.call('REASON', '', '', [''])


def generate_path_to_target(client, current_location, target_location, reachable_locations):
    """
    Generate a potential path from the current location to the target location.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        current_location (str): The current location of the robot.
        target_location (str): The target location to reach.
        reachable_locations (list): List of reachable locations from the current location.

    Returns:
        list: A potential path from the current location to the target location.
    """
    potential_path = []
    target_connectedTo_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['connectedTo', target_location])
    target_connectedTo = helper.list_Locations(target_connectedTo_query.queried_objects)
    common_location = helper.same_elements_bt_lists(target_connectedTo, reachable_locations)

    if common_location is None:
        update_location_property(client, 'Robot1', reachable_locations[0], current_location)
        current_location = reachable_locations[0]
        hena = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
        hena = helper.clean_list(hena.queried_objects)
        print('Robot here at:', hena)
        reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
        reachable_locations = helper.list_Locations(reachable_locations_query.queried_objects)
        common_location = helper.same_elements_bt_lists(target_connectedTo, reachable_locations)

    potential_path.append(common_location)
    potential_path.append(target_location)
    potential_path.append(current_location)

    return potential_path


def callback_map(data):
    """
    callback function for updating the published bool detects the status of 
    map_uploaded_flag (map uploaded or not).

    Args:
        data: the subscribed bool value.
        
    Returns:
        map_Uploaded_flag: The flag after changing it's status.
    """
    global map_Uploaded_flag
    if data.data == 1:
        map_Uploaded_flag= 1
    elif data.data ==0:
        map_Uploaded_flag = 0


def callback_bat(data):
    """
    callback function for updating the published bool detects the status of 
    battery_charged_flag (battery status).

    Args:
        data: the subscribed bool value.
        
    Returns:
        battery_charged_flag: The flag after changing its status.
    """
    global battery_charged_flag
    if data.data == 1:
        battery_charged_flag= 1
    elif data.data ==0:
        battery_charged_flag = 0


class loading_map(smach.State):
    """
    State class for the loading_map state in the finite state machine to wait in it until the map is built.

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['NOT_YET_UPLODED', 'UPLOADED'])

    def execute(self, userdata):
        global map_Uploaded_flag
        client = ArmorClient("example", "ontoRef")
        print(client)
        rospy.sleep(2)
        if map_Uploaded_flag==0:
            return 'NOT_YET_UPLODED'
        elif map_Uploaded_flag==1:
            client.call('LOAD','FILE','',[new_map, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            print("MAP IS LOADED SUCCESSFULLY")
            set_map_coordinates()
            return 'UPLOADED'   



class moving_in_corridoor_planning_for_urgent(smach.State):
    """
    State class for the moving_in_corridoor_planning_for_urgent state in the finite state machine for the robot to keep 
    moving in corridoor until there is urgent room detected.  

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_moving_in_cooridoor','urgent_room_exist','battery_low'])
        self.counter = 0

    def execute(self, userdata):
        global battery_charged_flag
        global urgent_room_flag 
        client = ArmorClient("example", "ontoRef")
        check_nearby_urgent_room()
        if battery_charged_flag==0:
            print("BATTERY LOW")
            return 'battery_low'

        elif battery_charged_flag==1 :
            if urgent_room_flag==1:
                print("urgent room detected")
                return 'urgent_room_exist'
            elif urgent_room_flag ==0:
                if random.randint(0, 1)==0:
                    navigate_to('C1')
                    print("im moving to c1")
                    rospy.sleep(0.5)
                else:
                    navigate_to('C2')
                    print("moving to c2")
                    rospy.sleep(0.5)
            return 'still_moving_in_cooridoor'
            
        #can be removed from here to the end of the 
        else:
            if random.randint(0, 1)==0:
                navigate_to('C1')
                print("im moving to c1")
                rospy.sleep(0.5)
            else:
                navigate_to('C2')
                print("moving to c2")
                rospy.sleep(0.5)
            return 'still_moving_in_cooridoor'
        

class move_to_urgent_room(smach.State):
    """
    State class for the move_to_urgent_room state in the finite state machine once the urgent is chosen.  

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_moving_to_urgent_rooms','cannot_plan','battery_low'])

    def execute(self, userdata):
        global battery_charged_flag
        global urgent_room_flag
        client = ArmorClient("example", "ontoRef")
        check_nearby_urgent_room()
        if battery_charged_flag==0:
            return 'battery_low'
        elif  battery_charged_flag==1:
            if urgent_room_flag==0:
                return 'cannot_plan'
            else:
                final_urgent_room=check_nearby_urgent_room()
                navigate_to(final_urgent_room)
                round_in_the_room()
                return 'still_moving_to_urgent_rooms'

        

class Recharging(smach.State):
    """
    State class for the Recharging state in the finite state machine for going to the charging 
    room to charge the robot's battery.

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_charging_battery','battery_charged'])

    def execute(self, userdata):
        
        global battery_charged_flag
        client = ArmorClient("example", "ontoRef")
        if battery_charged_flag==0:
            navigate_to('E')
            return 'still_charging_battery'
        elif battery_charged_flag==1: 
            return 'battery_charged'




def main():
    rospy.init_node('finite_state_machine')

    # Create a SMACH state machine
    R_FMS = smach.StateMachine(outcomes=[''])

    # Open the container
    with R_FMS:
        # Add states to the container
        smach.StateMachine.add('loading_map', loading_map(), 
                                transitions={'NOT_YET_UPLODED':'loading_map', 'UPLOADED':'moving_in_corridoor_planning_for_urgent'})
        smach.StateMachine.add('moving_in_corridoor_planning_for_urgent', moving_in_corridoor_planning_for_urgent(), 
                                transitions={'still_moving_in_cooridoor':'moving_in_corridoor_planning_for_urgent','urgent_room_exist':'move_to_urgent_room','battery_low':'Recharging'})
        smach.StateMachine.add('move_to_urgent_room', move_to_urgent_room(), 
                        transitions={'still_moving_to_urgent_rooms':'move_to_urgent_room','cannot_plan':'moving_in_corridoor_planning_for_urgent','battery_low':'Recharging'})
        smach.StateMachine.add('Recharging', Recharging(), 
                        transitions={'still_charging_battery':'Recharging','battery_charged':'moving_in_corridoor_planning_for_urgent' })



    rospy.Subscriber("load_map", Bool, callback_map)
    rospy.Subscriber("battery_state", Bool, callback_bat)
    rospy.wait_for_service('robot_coordinates')

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', R_FMS, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = R_FMS.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    
