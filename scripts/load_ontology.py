#!/usr/bin/env python

"""
.. module:: load_ontology
  :platform: Unix 
  :synopsis: Python module for loading the built map
.. moduleauthor:: Hussein Ahmed Fouad Hassan, S5165612@studenti.unige.it

The primary purpose of this module is to build the raw map which has only "Robot1"
elment with the whole data and object properties plus the coordinates of the locations.


"""

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
import assignment2.msg
from os.path import dirname, realpath
import helper


client = ArmorClient("example", "ontoRef") 

path = dirname(realpath(__file__))
# Put the path of the file.owl
oldontology = path + "/../maps/topological_map.owl"
new_map = path + "/../maps/new_map.owl"


ids=[]
def get_ids(string):

    """
    This function get the ids from the camera of the robot and update the global vatiable ids 
    which will be used to get the locations.

    Args:
        string (str):  the subcribed list of ids to be seperated and put in the ids.

    """

    global ids
    ids = list(set(ids + [int(lett) for lett in string.data.split() if lett.isdigit() and 10 < int(lett) < 18]))


    
def update_timestamp():
    """
    function for Updating the NOW property for the robot

    """
    res = client.call('QUERY','DATAPROP','IND',['now','Robot1'])
    old = str(helper.get_time(res.queried_objects))
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now','Robot1','Long',new,old])
    client.call('REASON','','',[''])



def load_map():
    """
    Builds the ontology according to the markers ids and save all the locations, rooms and corridors with their coordinates.

    """
    doors=[]
    client = ArmorClient("example", "ontoRef")
    client.call('LOAD','FILE','',[oldontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    doors = client.call('QUERY','IND','CLASS',['DOOR'])
    doors = helper.list_Locations(doors.queried_objects)
    print(doors)
    locations = client.call('QUERY','IND','CLASS',['LOCATION'])
    locations =helper.list_Locations(locations.queried_objects)
    print (locations)

    rospy.wait_for_service('/room_info')
    room_info = rospy.ServiceProxy('room_info', RoomInformation)
    #print (room_info)

    rospy.loginfo('Adding locations:')
    for id in ids:
        res = room_info(id)
        #print(res)
        room = res.room
        position_x = str(res.x)
        position_y = str(res.y)
        # if the room does not exist, add it
        if(locations.count(room) == 0):
            client.call('ADD','IND','CLASS',[room, 'LOCATION'])
            locations.append(room)
        # add the coordinates
        client.call('ADD','DATAPROP','IND',['X-coordinates', room, 'Float', position_x])
        client.call('ADD','DATAPROP','IND',['Y-coordinates', room, 'Float', position_y])

        # If room has multiple connections
        for i in res.connections:
            connected_location = i.connected_to
            connected_door = i.through_door
            
            # if the door does not exist, add it
            if(doors.count(i.through_door) == 0):
                client.call('ADD','IND','CLASS',[i.through_door, 'DOOR'])
                doors.append(i.through_door)
                client.call('ADD','OBJECTPROP','IND',['hasDoor', room, i.through_door])

            # if the location does not exist, add it
            if(locations.count(connected_location) == 0):
                client.call('ADD','IND','CLASS',[connected_location, 'LOCATION'])
                locations.append(connected_location)


    # All markers and ids are ready
    client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])
    for i in list(set(locations)):
        if i.startswith('R'):
            client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))

    #update timestamp
    update_timestamp()
    rooms = client.call('QUERY','IND','CLASS',['ROOM'])
    rooms = helper.list_Locations(rooms.queried_objects)
    print ("rooms quired")
    #print(rooms)
    corridors = client.call('QUERY','IND','CLASS',['CORRIDOR'])
    corridors = helper.list_Locations(corridors.queried_objects)
    rospy.loginfo('Locations:')
    rospy.loginfo(rooms)
    rospy.loginfo('of which, corridors:')
    rospy.loginfo(corridors)

    # At first the robot is in the recharging room
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1','E'])
    # Update its visitedAt
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old=helper.get_time(req.queried_objects)
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['visitedAt','E','Long',new,old])
    #update timestamp
    update_timestamp() 
    client.call('SAVE','','',[new_map])
    print ("i reached the end")


def main():
   global ids
   rospy.init_node('load_map_node', anonymous=True)
   #subscriber=rospy.Subscriber("/marker_publisher/marker_pub_topic", String, get_ids)
   while True:
      #print("waiting for ids")
      ids=[11, 12, 13, 14, 15, 16, 17]
      print(ids)
      if len(ids)>=7:
        print("I have all the markers, I will build the map")
        #subscriber.unregister()
        pub = rospy.Publisher('load_map', Bool, queue_size=10)
        pub.publish(0)
        load_map()
        print("Map built")
        pub.publish(1)
        rospy.sleep(3)
        rospy.signal_shutdown('load_map_node')
        rospy.signal_shutdown('marker_publisher')

      rospy.sleep(1)


rospy.sleep(3)
   
if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass 



