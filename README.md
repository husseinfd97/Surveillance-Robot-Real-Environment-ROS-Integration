# Second Assignment of Experimental Robotic Laboratory 

This assignment is imporvement of the [first asssignment](https://github.com/husseinfd97/as_1.git) project which is focusing on implementing a robotic system using the Armor package which is resposible for manipulating online OWL ontologies and Smach package which is resposible for building hierarchical state machines decribing the diffrent staes of the robot. The system aims to simulate a robot operating in a 2D environment consisting of four rooms and three corridors. The robot's behavior is designed to navigate through the environment, visiting different locations while following certain policies but in this assignment URDF of a robot and the code is implemented in real environment as each room and location has real x and y coordinates and here you can find the [documented](https://husseinfd97.github.io/assignment2/) code with sphinx.


## Environment
The 3D environment is composed of four rooms and three corridors. The rooms are labeled as R1, R2, R3, and R4, while the corridors are labeled as C1, C2, and E. The robot moves within this environment, with various doors (D1...D6) connecting the rooms and corridors as it's shown in the figure.

![map](https://github.com/husseinfd97/assignment2/assets/94136236/0af09991-1499-45d3-b3e1-6fd2faa056ba)


The indoor environment comprises various entities, such as doors, rooms, and corridors, with interconnected relationships . When two locations share a common door, it implies a connection between them denoted by the "connectedTo" relation. Furthermore, rooms that have not been visited for a specified duration, defined by the urgency threshold, are designated as urgent. These relationships and urgency status provide important contextual information about the environment, facilitating navigation and decision-making processes for the robot and you can find the pre-build map with gazebo (not deteced by the system yet) in the folder `world`.
## Robot model

![robot](https://github.com/husseinfd97/assignment2/assets/94136236/92df9d8a-1b79-4131-8b33-d41f9745b8a2)

The model concists of four-wheeled mobile robot conncected together with the base frame, mounted in the base phare arm robot has revolute joint can be controlled for the scanning missions and at the end of this arm there is fixed camera which will be used to detect the markers.

## Robot Behavior and Logic
The robot's behavior can be divided into two phases:

### Phase 1: Building the Topological Map
The robot starts in the initial position as the robot get spawned in the position x = -6.0, y = 11.0 and then without moving the base of the robot the arm should rotate to scan the initial to and detect the markers with `openCV` library which have this information includes the relations between C1, C2, R1, R2, R3 locations, the doors D1...D6 and the coordinates of each room and corridor.

### Phase 2: Navigating and Visiting Locations
After building the topological map, the robot moves to a new location and waits for some time before visiting another location by setting the goal as the x and y coordinates of the next position and send it to `move_base` library . This behavior is repeated in an infinite loop. When the robot's battery is low it cancels the whole goals for the robot and it returns to the E location and waits for the charging time before starting the behavior again.

The surveillance policy for the robot's movement is as follows:
- When the robot's battery is not low, it primarily stays on corridors.
- If a reachable room has not been visited for some time, the robot should visit it.

The implemented code ensures that the robot follows this behavior and policy by incorporating the necessary functions and logic.

## Software Architecture
### Node diagram
![arc](https://github.com/husseinfd97/assignment2/assets/94136236/e6fdb8aa-b104-4bb0-b147-1b75bfedd4b8)



The software architecture of the project consists of three nodes:

1. Load Map Node: Responsible for loading the topological map and publishing the load_map status and subscribes the markers ids after detection using openCV to build the map.
2. State Machines Node: Implements the finite state machine (FSM) that controls the robot's behavior and actions. It subscribes to the load_map and battery_state topics, updates the urgent_room_flag and location_updated flags, and performs the necessary actions such as moving in the corridor, moving to urgent rooms, and recharging plus it send the /robot_coordinates x and y using service robot_coordinates.srv.
3. Battery Node: Publishes the battery_state status periodically, indicating the current battery level of the robot.
4. Marker publisher: using openCV library get the data from the camera of the robot after detecting the numbers from the markers then publish the list of the numbers of the markers to load_map node.
5. Marker server Node: getting the markers numbers from load_map node then gives it the coordinates of each room and corridor using service RoomInformation.srv .
6. Autonomous Node: gets the coordinates of rooms which will be visted now and give it to move_base node to set a goal and stop the whole code until it reaches this goal.
7. Scanner Node: moves the arm robot by controlling the rovolute joint to let the camera detects the whole markers.
   
### Temporal Diagram
A temporal diagram illustrates the sequence of interactions between the states, showcasing how the load_map, urgent_room_flag, location_updated, and battery_state are published and utilized by the state machines node.

![tem](https://github.com/husseinfd97/assignment2/assets/94136236/5311d91d-785c-4461-accc-8fab9e464575)


### State Diagrams
State diagrams provide a visual representation of the different states and transitions in the state machines node. They illustrate the flow and decision-making process of the finite state machine.

![state](https://github.com/husseinfd97/assignment2/assets/94136236/221586c4-70f0-4389-82bb-486df4c150ab)

### Running Code



And here you can find the pseudocode describing the most important two functions used for surveillance policy:
```
function check_nearby_urgent_room():
    set urgent_room_flag to 0
    set least_room to None

    # Perform necessary queries
    query urgent_rooms
    query current_location

    if current_location is 'C1':
        query room1_visit_time and room2_visit_time
        calculate least_room
    else if current_location is 'C2':
        query room3_visit_time and room4_visit_time
        calculate least_room

    find matching room in urgent_rooms_query

    if matching room is found:
        update urgent_room_flag
        return corresponding room

    return "0"

```
```
function navigate_to(target_location):
    # Perform necessary queries
    query initial_location
    if target_location is equal to current_location:
        if the robot reached the coordinates:
            update location properties
            check and update visitedAt property
    else:
        query reachable_locations

        if target_location is in reachable_locations:
               if the robot reached the coordinates:
                    update location properties
                    check and update visitedAt property
        else:
            generate potential path
            extract intermediate_location, current_location, and target_location
            if the robot reached the coordinates:
                update location properties
                query current_location
                update now property
                check and update visitedAt property
                update current_location

            query reachable_locations

            if target_location is in reachable_locations:
                if the robot reached the coordinates:
                      update location properties
                      check and update visitedAt property
```


## Dependencies

In order to run the simulation, this software is designed to be compatible with the [ROS Noetic](http://wiki.ros.org/noetic) environment. To ensure smooth execution, it is necessary to have a properly initialized ROS workspace. Additionally, there are several required packages that need to be installed to support the functionality of the software
  - [roslaunch](http://wiki.ros.org/roslaunch), to launch the files in the package.
  - [rospy](http://wiki.ros.org/rospy), to use python with ROS.
  - [actionlib](http://wiki.ros.org/actionlib/DetailedDescription), which will be used to set the coordinates.
  - [xterm](https://wiki.archlinux.org/title/Xterm), a terminal simulator, which can be installed by running from the terminal `sudo apt install -y xterm`.
  - [ARMOR Server](https://github.com/EmaroLab/armor), is a ROS integration facilitates the manipulation of OWL ontologies online.
  - [smach](http://wiki.ros.org/smach), To enable simulation of the robot's behavior, a state machine library is utilized, which can be installed by excuting this line in the terminal `sudo apt-get install ros-noetic-smach-ros`
  - [OpenCV](http://wiki.ros.org/vision_opencv), a library for image processing.
  - [move_base](http://wiki.ros.org/move_base), to control the robot motion allover the map.
  - [gmapping](http://wiki.ros.org/gmapping), implements the SLAM algorithm to build the map based on the laser scan.
  - [rviz](http://wiki.ros.org/rviz), a robotic visualization tool.
  - [gazebo](https://gazebosim.org/), a simulation tool.

## Installation and Running Procedure
To install and run the project, follow these steps:
1. Navigate to your workspace's src directory.
2. Clone the project repository: `git clone https://github.com/husseinfd97/as_1.git`
3. Build the project: `catkin_make`
4. go to scripts folder and make all files are excutable with `chmod +x name_of_the_file.py`
5. Launch the project: `roslaunch assignment2 simulation_and_move_base.launch` `roslaunch assignment2 scanner_and_detection.launch`  `roslaunch assignment2 fms.launch`

## System's Limitations
While the implemented system covers the specified requirements, it also has some limitations:
- The surveillance policy is simple and does not consider dynamic changes and designed only for this fixed map.
- The battery status change doesn't depend on how far the robot went, it's generated only with time.
- The system is a bit slow and it can lead to some errors with move_base node.

## Possible Technical Improvements
To enhance the system, the following improvements can be considered:
- Implementing more sophisticated surveillance policies that account for dynamic factors and optimize robot movement (for maps more than 2 corridors and dynamically changing).
- Design a battery system depends on feedback from the robot depedning on how far the robot went.

  By addressing these improvements, the system can be further optimized for various real-world scenarios and requirements.

***Author***: Hussein Ahmed Fouad Hassan

***Email***: S5165612@studenti.unige.it

