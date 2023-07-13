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
import assignment2.msg


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

    
def update_timestamp():
    res = client.call('QUERY','DATAPROP','IND',['now','Robot1'])
    old = str(findtime(res.queried_objects))
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now','Robot1','Long',new,old])
    client.call('REASON','','',[''])



def load_map():
    """
    Popolates the ontology according to the markers ids and save the names of the robot and all the locations, rooms and corridors
    """
    doors=[]
    client = ArmorClient("example", "ontoRef")
    client.call('LOAD','FILE','',[oldontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    #doors = clean_list(client.call('QUERY','IND','CLASS',['DOOR']))
    doors = client.call('QUERY','IND','CLASS',['DOOR'])
    doors = list_Locations(doors.queried_objects)
    print(doors)
    locations = client.call('QUERY','IND','CLASS',['LOCATION'])
    locations =list_Locations(locations.queried_objects)
    print (locations)

    rospy.wait_for_service('/room_info')
    room_info = rospy.ServiceProxy('room_info', RoomInformation)
    #print ('a7a')
    #print (room_info)

    rospy.loginfo('Adding locations:')
    for id in ids:
        res = room_info(id)
        #print(res)
        room = res.room;
        #print('room')
        #print(room)
        #print(res.x)
        #print(res.y)
        position_x = str(res.x);
        position_y = str(res.y);
        # if the room does not exist, add it
        if(locations.count(room) == 0):
            client.call('ADD','IND','CLASS',[room, 'LOCATION'])
            locations.append(room)
        # add the coordinates
        client.call('ADD','DATAPROP','IND',['X-coordinates', room, 'Float', position_x])
        client.call('ADD','DATAPROP','IND',['Y-coordinates', room, 'Float', position_y])
        # last visitedAt = 0
        #time_zero = str(0)
        #client.call('ADD','DATAPROP','IND',['visitedAt',room,'Long',time_zero])
        
        #client.call('REPLACE','DATAPROP','IND',['visitedAt',room,'Long',time_zero,time_zero])
        # if id.startswith('R'):
        #     client.manipulation.add_dataprop_to_ind("visitedAt", id, "Long", str(math.floor(time.time())))
        
    
    
    
        #client.call('REPLACE','DATAPROP','IND',['visitedAt',room,'Long',new,old])

        # rooms may have multiple connections
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


    # here I should have the complete ontology, 7 ids for 7 markers
    #client.call('DISJOINT','IND','CLASS',['LOCATION'])
    #client.call('DISJOINT','IND','CLASS',['DOOR'])
    client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])
    #client.call('DISJOINT','IND','',list(set(doors)))
    #client.call('REASON','','',[''])
    for i in list(set(locations)):
        if i.startswith('R'):
            client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))

    #update timestamp
    update_timestamp()

    #SAVE THE CONSTANTS LISTS(((alaaaaaaaaaaaaaaaaaaarrrmmmm it doesn't do the sipiration)))
    rooms = client.call('QUERY','IND','CLASS',['ROOM'])
    rooms = list_Locations(rooms.queried_objects)
    print ("rooms quired")
    print(rooms)
    corridors = client.call('QUERY','IND','CLASS',['CORRIDOR'])
    corridors = list_Locations(corridors.queried_objects)
    print ("corridor quired")
    print(corridors)
    rospy.loginfo('Locations:')
    rospy.loginfo(rooms)
    rospy.loginfo('of which, corridors:')
    rospy.loginfo(corridors)

    # At first the robot is in the recharging room
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1','E'])
    # Update its visitedAt
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old=findtime(req.queried_objects)
    new = str(round(time.time()))
    client.call('REPLACE','DATAPROP','IND',['visitedAt','E','Long',new,old])
    #client.call('REASON','','',[''])

    #update timestamp
    update_timestamp()
    
    
    
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



