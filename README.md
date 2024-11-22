# PDE4430_MOBILE_ROBOTICS
________________________________________
PDE4430 Mobile Robotics - Coursework 1
A ROS-based project that demonstrates efficient robot control and navigation in the Turtlesim environment. The program implements various tasks for autonomous and coordinated multi-robot behaviours.
________________________________________
Table of Contents
1.	Project Overview
2.	Features
3.	Setup Instructions
4.	Usage
5.	Design and Architecture
________________________________________
Project Overview
This project utilizes the Robotic Operating System (ROS) and the turtlesim simulator to showcase meaningful movement and coordination of one or more robots. The project includes teleoperation, autonomous navigation, collision avoidance, and multi-robot behaviours for efficient space coverage.
________________________________________
Features
•	Teleoperation Mode: Control the robot using the keyboard, with adjustable movement speed.
•	Autonomous Navigation: Navigate to a specified coordinate within the Turtlesim window.
•	Collision Avoidance: Override movements to prevent collisions with walls.
•	Vacuum Cleaning Behavior: Efficiently cover the entire window area with a single robot.
•	Multi-Robot Coordination: Manage multiple turtles (2 or more) working together for vacuum cleaning.
________________________________________
Setup Instructions
1.	Clone the Repository:
2.	git clone <repository_url>
3.	cd <repository_name>
4.	Copy the Package to Your Workspace:
5.	cp -r <repository_name> ~/catkin_ws/src/
6.	cd ~/catkin_ws
7.	Build the Workspace:
8.	catkin_make
9.	source devel/setup.bash
10.	Verify Installation:
11.	rospack find <package_name>
________________________________________
Usage
1. Start ROS Master:
roscore
2. Launch the Turtlesim Node:
rosrun turtlesim turtlesim_node
3. Run the Project Scripts:
•	Teleoperation: 
•	rosrun teleop teleoperation.py
•	Autonomous Navigation: 
•	rosrun turtle_nav autonomous_navigation.py
•	Wall Avoidance: 
•	rosrun turtle_wall_avoidance wall_avoidance.py
•	Vacuum Cleaning: 
•	rosrun vacuum_clean vacuum_turtle.py
•	Multi-Turtle Cleaning: 
•	rosrun multi_turtle multi_turtle.py
________________________________________
Design and Architecture
Nodes and Topics
•	Nodes: 
o	teleoperation.py: Handles keyboard control.
o	autonomous_navigation.py: Navigates to specified coordinates.
o	wall_avoidance.py: Stops the turtle movement when wall detected.
o	vacuum_turtle.py: Implements vacuum cleaning for a single turtle.
o	multi_turtle.py: Coordinates multiple turtles.
•	Topics: 
o	/turtle1/cmd_vel: Publishes velocity commands.
o	/turtle1/pose: Subscribes to the turtle's position for navigation and collision avoidance.
o	/spawn: Service call to spawn additional turtles.
Algorithm Flow
1.	Input Handling: 

o	Teleoperation for keyboard-based control.
o	Coordinates for autonomous navigation.

2.	Collision Avoidance: 

o	Monitors the turtle's position and overrides movement commands to avoid collisions.

3.	Multi-Turtle Coordination: 

o	Spawns additional turtles and coordinates their movements to cover the area efficiently.

4.	Vacuum Cleaning: 

o	Implements grid or spiral patterns to maximize area coverage.

RQT Graph
Include an RQT graph here to visualize nodes, topics, and their interactions.
________________________________________
Author
Created as part of PDE4430 Mobile Robotics - Middlesex University Dubai coursework.
Email: zainabkazi.zk@gmail.com

________________________________________

