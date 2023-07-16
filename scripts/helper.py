#!/usr/bin/env python
"""
.. module:: helper
  :platform: Unix 
  :synopsis: Python module for having extra functions needed in the system
.. moduleauthor:: Hussein Ahmed Fouad Hassan, S5165612@studenti.unige.it


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
import datetime



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

# def findtime(list):
#     """
#     Function for finding the time in Unix format from the returned query property from armor.

#     Args:
#         list (list): The time in the armor response format, e.g., ['"1669241751"^^xsd:long']

#     Returns:
#         str: The extracted time as a string, e.g., "1665579740"
#     """
#     for i in list:
#         try:
#             start = i.index('"') + len('"')
#             end = i.index('"', start)
#             return i[start:end]
#         except ValueError:
#             pass
#     return ""

def get_time(list):
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

def same_elements_bt_lists(l1, l2):
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

def get_least_visit_time_room(*args):
    """
    Get the room with the least visit time from the given room-time pairs.

    Args:
        args: Alternating arguments of room and time pairs. The time value must be a number.

    Returns:
        str: The room with the least visit time.
    """
    visit_times = {}

    # Iterate over the alternating room-time pairs
    for i in range(0, len(args), 2):
        room = args[i]
        time = args[i + 1]
        visit_times[room] = time

    min_visit_time = min(visit_times.values())
    least_visited_room = min(visit_times, key=visit_times.get)

    return least_visited_room
