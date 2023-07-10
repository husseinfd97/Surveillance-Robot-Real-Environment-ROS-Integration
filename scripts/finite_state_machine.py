#!/usr/bin/env python


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





#########
#get the same way in upload map 
path = dirname(realpath(__file__))
new_map = path + "/../maps/new_map.owl"

 
######### 

map_Uploaded_flag=1
battery_charged_flag=1
urgent_room_flag=0
planning_finished_flag=1
coordinates=[]




def move_base(desired):
    """
    Move the robot to the specified location using the move_base function.

    Args:
        desired (str): The name of the location to move the robot to.

    Returns:
        bool: True if the target was reached successfully, False otherwise.
    """
    rospy.wait_for_service('robot_coordinates')  # Wait for the move_base service to become available
    try:
        move_base_service = rospy.ServiceProxy('robot_coordinates', robot_coordinates )
        response = move_base_service(coordinates[desired]['X'], coordinates[desired]['Y'])
        if response.success:
            print("Target reached successfully!")
            return True
        else:
            print("Target not reached. Retrying...")
            return move_base(desired)  # Retry the move_base call
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))
        return False


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
    print("Scanning...")
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

def set_coordinates():
    """
    Sets the global variable 'coordinates' to a dictionary of the locations and their corresponding X and Y coordinates.

    First, a list of room names is created. Then, for each room in the list, the Xcoordinates and Ycoordinates of the room are queried using the ArmorClient and stored as variables 'X' and 'Y', respectively. These values are then added to the 'coordinates' dictionary as a key-value pair with the room name as the key and a dictionary of the X and Y coordinates as the value.

    Args:
        void

    Returns:
        void
    """
    global coordinates
    client = ArmorClient("example", "ontoRef")
    list_of_rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    for i in list_of_rooms:
        req=client.call('QUERY','DATAPROP','IND',['Xcoordinates', i])
        X=float(findbt(req.queried_objects))
        req=client.call('QUERY','DATAPROP','IND',['Ycoordinates', i])
        Y=float(findbt(req.queried_objects))
        coordinates[i] = {'X': X, 'Y': Y}

def clean_list(list):
    """
    Function for finding the individual in a list from the returned query property from armor.

    Args:
        list (list): The individual in the armor response format, e.g., ['<http://bnc/exp-rob-lab/2022-23#E>']

    Returns:
        str: The extracted individual as a string, e.g., "E"
    """
    individuals = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    for i in list:
        for individual in individuals:
            if individual in i:
                return individual
    return ""

def findtime(list):
    """
    Function for finding the time in Unix format from the returned query property from armor.

    Args:
        list (list): The time in the armor response format, e.g., ['"1669241751"^^xsd:long']

    Returns:
        str: The extracted time as a string, e.g., "1665579740"
    """
    for i in list:
        try:
            start = i.index('"') + len('"')
            end = i.index('"', start)
            return i[start:end]
        except ValueError:
            pass
    return ""

def findbt(list):
    """
    Function for extracting data between quotation marks from a list. 

    Args:
        lst (list): A list containing strings with data enclosed in quotation marks. 

    Returns:
        str: The extracted data. 
    """
    for i in list:
        try:
            start = i.index('"') + len('"')
            end = i.index('"', start)
            return i[start:end]
        except ValueError:
            return ""

def list_Locations(list):
    """
    Function for extracting the locations from a list of query properties.

    Args:
        lst (list): A list containing query properties, e.g., ['<http://bnc/exp-rob-lab/2022-23#R1>', '<http://bnc/exp-rob-lab/2022-23#R2>']

    Returns:
        list: The extracted locations, e.g., ['R1', 'R2']
    """
    position_list = []
    for i in list:
        if "R1" in i:
            position_list.append('R1')
        elif "R2" in i:
            position_list.append('R2')
        elif "R3" in i:
            position_list.append('R3')
        elif "R4" in i:
            position_list.append('R4')
        elif "C1" in i:
            position_list.append('C1')
        elif "C2" in i:
            position_list.append('C2')
        elif "E" in i:
            position_list.append('E')
    return position_list

def find_common_connection(l1, l2):
    """
    Function for finding a common connection between two lists.

    Args:
        l1 (list): The first list.
        l2 (list): The second list.

    Returns:
        str: The common connection between the two lists, if found.
    """
    for common in l1:
        if common in l2:
            return common


def check_nearby_urgent_room():
    """
    Check for nearby urgent rooms, update the global `urgent_room_flag`, and return the urgent room if it exists.

    Returns:
        str: The urgent room nearby, according to the urgency of each robot. Returns '0' if no urgent room is found.
    """
    global urgent_room_flag
    urgent_room_flag = 0  # Initially, assume there's no urgent room
    client = ArmorClient("example", "ontoRef")
    client.call('REASON', '', '', [''])

    # Query for urgent rooms
    urgent_rooms_query = client.call('QUERY', 'IND', 'CLASS', ['URGENT'])
    urgent_rooms_query = list_Locations(urgent_rooms_query.queried_objects)
    
    # Query current robot location
    current_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    current_location = clean_list(current_location_query.queried_objects)

    for room in urgent_rooms_query:
        # Query visit times for each room
        room1_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R1'])
        room2_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R2'])
        room3_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R3'])
        room4_visit_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', 'R4'])

        room1_visit_time = float(findbt(room1_visit_time_query.queried_objects))
        room2_visit_time = float(findbt(room2_visit_time_query.queried_objects))
        room3_visit_time = float(findbt(room3_visit_time_query.queried_objects))
        room4_visit_time = float(findbt(room4_visit_time_query.queried_objects))

        if "R1" in room and room2_visit_time - room1_visit_time <= 0:
            urgent_room_flag = 1
            return 'R1'
        elif "R2" in room and room1_visit_time - room2_visit_time < 0:
            urgent_room_flag = 1
            return 'R2'
        elif "R3" in room and room4_visit_time - room3_visit_time <= 0:
            urgent_room_flag = 1
            return 'R3'
        elif "R4" in room and room3_visit_time - room4_visit_time < 0:
            urgent_room_flag = 1
            return 'R4'

    return '0'  # No nearby urgent room found


def moveto(target_location):
    """
    Move the robot to the target location.

    Args:
        target_location (str): The location to which the robot should move.
    """
    client = ArmorClient("example", "ontoRef")
    client.call('REASON', '', '', [''])

    # Query initial robot location
    initial_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    current_location = clean_list(initial_location_query.queried_objects)
    print('From', current_location, 'to:', target_location)

    if target_location == current_location:
        # If the target location is the same as the current location, directly move to the target location
        update_location_property(client, 'Robot1', target_location, current_location)
        update_now_property(client, 'Robot1')
        check_and_update_visitedat_property(client, target_location)
    else:
        reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
        reachable_locations = list_Locations(reachable_locations_query.queried_objects)
        
        if target_location in reachable_locations:
            update_location_property(client, 'Robot1', target_location, current_location)
            update_now_property(client, 'Robot1')
            check_and_update_visitedat_property(client, target_location)
        else:
            potential_path = generate_path_to_target(client, current_location, target_location, reachable_locations)
            intermediate_location = potential_path[0]
            current_location = potential_path[2]
            update_location_property(client, 'Robot1', intermediate_location, current_location)
            hena = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
            hena = clean_list(hena.queried_objects)
            print('Robot here at:', hena)
            update_now_property(client, 'Robot1')
            check_and_update_visitedat_property(client, intermediate_location)
            current_location = intermediate_location

            reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
            reachable_locations = list_Locations(reachable_locations_query.queried_objects)

            if target_location in reachable_locations:
                update_location_property(client, 'Robot1', target_location, current_location)
                update_now_property(client, 'Robot1')
                check_and_update_visitedat_property(client, target_location)

    final_location_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
    print('Robot finally isIn', clean_list(final_location_query.queried_objects))


# def update_location_property(client, robot, new_location, old_location):
#     """
#     Update the location property of the robot.

#     Args:
#         client (ArmorClient): The ArmorClient object for making calls to the server.
#         robot (str): The name of the robot.
#         new_location (str): The new location to update.
#         old_location (str): The old location to replace.
#     """
#     client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', robot, new_location, old_location])
#     client.call('REASON', '', '', [''])
def update_location_property(client, robot, new_location, old_location):
    """
    Update the location property of the robot.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        robot (str): The name of the robot.
        new_location (str): The new location to update.
        old_location (str): The old location to replace.
    """

    rt=move_base(new_location)
    if rt == 1:
        client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', robot, new_location, old_location])
        client.call('REASON', '', '', [''])

def update_now_property(client, robot):
    """
    Update the current time property of the robot.

    Args:
        client (ArmorClient): The ArmorClient object for making calls to the server.
        robot (str): The name of the robot.
    """
    current_time_query = client.call('QUERY', 'DATAPROP', 'IND', ['now', robot])
    current_time = findbt(current_time_query.queried_objects)
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
        visited_time = findbt(visited_time_query.queried_objects)
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
    target_connectedTo = list_Locations(target_connectedTo_query.queried_objects)
    common_location = find_common_connection(target_connectedTo, reachable_locations)

    if common_location is None:
        update_location_property(client, 'Robot1', reachable_locations[0], current_location)
        current_location = reachable_locations[0]
        hena = client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
        hena = clean_list(hena.queried_objects)
        print('Robot here at:', hena)
        reachable_locations_query = client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])
        reachable_locations = list_Locations(reachable_locations_query.queried_objects)
        common_location = find_common_connection(target_connectedTo, reachable_locations)

    potential_path.append(common_location)
    potential_path.append(target_location)
    potential_path.append(current_location)

    return potential_path








def callback_map(data):

    global map_Uploaded_flag
    if data.data == 1:
        map_Uploaded_flag= 1
    elif data.data ==0:
        map_Uploaded_flag = 0


def callback_bat(data):
    global battery_charged_flag
    if data.data == 1:
        battery_charged_flag= 1
    elif data.data ==0:
        battery_charged_flag = 0


class loading_map(smach.State):
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
            set_coordinates()
            return 'UPLOADED'   



class moving_in_corridoor_planning_for_urgent(smach.State):
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

        elif battery_charged_flag==1 and urgent_room_flag==1:
            if urgent_room_flag==1:
                print("urgent room detected")
                return 'urgent_room_exist'
            elif urgent_room_flag ==0:
                if random.randint(0, 1)==0:
                    moveto('C1')
                    print("im moving to c1")
                    rospy.sleep(0.5)
                else:
                    moveto('C2')
                    print("moving to c2")
                    rospy.sleep(0.5)
            return 'still_moving_in_cooridoor'
            
        #can be removed from here to the end of the 
        else:
            if random.randint(0, 1)==0:
                moveto('C1')
                print("im moving to c1")
                rospy.sleep(0.5)
            else:
                moveto('C2')
                print("moving to c2")
                rospy.sleep(0.5)
            return 'still_moving_in_cooridoor'
        

class move_to_urgent_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['keep_moving_to_urgent_rooms','cannot_plan','battery_low'])

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
                moveto(final_urgent_room)
                round_in_the_room()
                return 'keep_moving_to_urgent_rooms'

        

class Recharging(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_charging_battery','battery_charged'])


    def execute(self, userdata):
        
        global battery_charged_flag
        client = ArmorClient("example", "ontoRef")
        if battery_charged_flag==0:
            moveto('E')
            return 'still_charging_battery'
        elif battery_charged_flag==1: 
            return 'battery_charged'




def main():
    rospy.init_node('finite_state_machine')

    # Create a SMACH state machine
    R_FMS = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with R_FMS:
        # Add states to the container
        smach.StateMachine.add('loading_map', loading_map(), 
                                transitions={'NOT_YET_UPLODED':'loading_map', 'UPLOADED':'moving_in_corridoor_planning_for_urgent'})
        smach.StateMachine.add('moving_in_corridoor_planning_for_urgent', moving_in_corridoor_planning_for_urgent(), 
                                transitions={'still_moving_in_cooridoor':'moving_in_corridoor_planning_for_urgent','urgent_room_exist':'move_to_urgent_room','battery_low':'Recharging'})
        smach.StateMachine.add('move_to_urgent_room', move_to_urgent_room(), 
                        transitions={'keep_moving_to_urgent_rooms':'move_to_urgent_room','cannot_plan':'moving_in_corridoor_planning_for_urgent','battery_low':'Recharging'})
        smach.StateMachine.add('Recharging', Recharging(), 
                        transitions={'still_charging_battery':'Recharging','battery_charged':'moving_in_corridoor_planning_for_urgent' })



    rospy.Subscriber("load_map_node", Bool, callback_map)
    rospy.Subscriber("battery_state", Bool, callback_bat)
    rospy.wait_for_service('robot_coordinates')


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', R_FMS, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = R_FMS.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    