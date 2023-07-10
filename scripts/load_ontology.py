#!/usr/bin/env python

# Import the armor client class
import time
from armor_client import ArmorClient
import random
import time
import math
import rospy
import rospkg
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool
from std_msgs.msg import Bool, String
from assignment2.srv import RoomInformation

from os.path import dirname, realpath
client = ArmorClient("example", "ontoRef") 


#def LoadMap():
path = dirname(realpath(__file__))
# Put the path of the file.owl
oldontology = path + "/../maps/topological_map.owl"
new_map = path + "/../maps/new_map.owl"

ids=[]


def get_ids(string):
    global ids
    ids = list(set(ids + [int(lett) for lett in string.data.split() if lett.isdigit() and 10 < int(lett) < 18]))




def clean_list(list):
    """
    Function for finding the individuals in a list from the returned query property from Armor.

    Args:
        list (list): The individuals in the Armor response format, e.g., ['<http://bnc/exp-rob-lab/2022-23#E>']

    Returns:
        list: The extracted individuals as a list of strings, e.g., ["E"]
    """
    individuals = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    cleaned_list = []
    for i in list:
        for individual in individuals:
            if individual in i:
                cleaned_list.append(individual)
    return cleaned_list

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

    


def load_map():
    """
    Popolates the ontology according to the markers ids and save the names of the robot and all the locations, rooms and corridors
    """
    
    client = ArmorClient("example", "ontoRef")
    client.call('LOAD','FILE','',[oldontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    #doors = clean_list(client.call('QUERY','IND','CLASS',['DOOR']))
    doors = client.call('QUERY','IND','CLASS',['DOOR'])
    doors = clean_list(doors.queried_objects)
    print(doors)
    locations = client.call('QUERY','IND','CLASS',['LOCATION'])
    locations =clean_list(locations.queried_objects)
    print (locations)

    rospy.wait_for_service('/room_info')
    room_info = rospy.ServiceProxy('room_info', RoomInformation)
    #print ('a7a')
    #print (room_info)

    rospy.loginfo('Adding locations:')
    for id in ids:
        res = room_info(id)
        #print(res)
        room = res.room
        print('room')
        print(res.x)
        print(res.y)
        position_x = str(res.x)
        position_y = str(res.y)
        # if the room does not exist, add it
        if(locations.count(room) == 0):
            client.call('ADD','IND','CLASS',[room, 'LOCATION'])
            locations.append(room)
        # add the coordinates
        client.call('ADD','DATAPROP','IND',['position_x', room, 'Float', position_x])
        client.call('ADD','DATAPROP','IND',['position_y', room, 'Float', position_y])
        # last visitedAt = 0
        time_zero = str(0)
        client.call('ADD','DATAPROP','IND',['visitedAt',room,'Long',time_zero])

        # rooms may have multiple connections
        for x in res.connections:
            connected_location = x.connected_to
            connected_door = x.through_door

            # if the door does not exist, add it
            if(doors.count(connected_door) == 0):
                client.call('ADD','IND','CLASS',[connected_door, 'DOOR'])
                doors.append(connected_door)
            client.call('ADD','OBJECTPROP','IND',['hasDoor', room, connected_door])

            # if the location does not exist, add it
            if(locations.count(connected_location) == 0):
                client.call('ADD','IND','CLASS',[connected_location, 'LOCATION'])
                locations.append(connected_location)

    # here I should have the complete ontology, 7 ids for 7 markers
    client.call('DISJOINT','IND','CLASS',['LOCATION'])
    client.call('DISJOINT','IND','CLASS',['DOOR'])

    #update timestamp
    res = client.call('QUERY','DATAPROP','IND',['now','Robot1'])
    old = str(findtime(res.queried_objects))
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now','Robot1','Long',new,old])
    client.call('REASON','','',[''])

    #SAVE THE CONSTANTS LISTS(((alaaaaaaaaaaaaaaaaaaarrrmmmm it doesn't do the sipiration)))
    rooms = client.call('QUERY','IND','CLASS',['ROOM'])
    rooms = clean_list(rooms.queried_objects)
    print ("rooms quired")
    print(rooms)
    # corridors = client.call('QUERY','IND','CLASS',['CORRIDOR'])
    # corridors = clean_list(corridors.queried_objects)
    # print ("corridor quired")
    # print(corridors)
    rospy.loginfo('Locations:')
    rospy.loginfo(rooms)
    # rospy.loginfo('of which, corridors:')
    # rospy.loginfo(corridors)

    # At first the robot is in the recharging room
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1','E'])
    # Update its visitedAt
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old=findtime(req.queried_objects)
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['visitedAt','E','Long',new,old])

    #update timestamp
    res = client.call('QUERY','DATAPROP','IND',['now','Robot1'])
    old = str(findtime(res.queried_objects))
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now','Robot1','Long',new,old])
    client.call('REASON','','',[''])
    
    
    
    client.call('SAVE','','',[new_map])
    print ("i reached the end")


def main():
   """
   The main function of the script. It initializes a ROS node, creates a subscriber and a publisher, and sets up a service client. It then enters a loop that waits for a message to be received on the subscriber, processes the message, and publishes a response. The loop also sleeps for a random amount of time before processing the next message.

   Args:
   None

   Returns:
   None
   """
   global ids
   rospy.init_node('load_map_node', anonymous=True)
   subscriber=rospy.Subscriber("/marker_publisher/marker_pub_topic", String, get_ids)
   while True:
      print("waiting for ids")
      #ids=[11, 12, 13, 14, 15, 16, 17]
      print(ids)
      if len(ids)>=7:
        print("I have all the markers, I will build the map")
        subscriber.unregister()
        pub = rospy.Publisher('load_map_pub', Bool, queue_size=10)
        pub.publish(0)
        load_map()
        print("Map built, I will shutdown the node")
        pub.publish(1)
        rospy.sleep(3)
        rospy.signal_shutdown('load_map_node')
      rospy.sleep(1)





   
rospy.sleep(3)
   
if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass 



