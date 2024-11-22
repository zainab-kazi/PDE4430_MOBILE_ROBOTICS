### **README Content**

```markdown
# PDE4430_MOBILE_ROBOTICS

---

## PDE4430 Mobile Robotics - Coursework 1

A ROS-based project that demonstrates efficient robot control and navigation in the Turtlesim environment. 
The program implements various tasks for autonomous and coordinated multi-robot behaviours.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Features](#features)
3. [Setup Instructions](#setup-instructions)
4. [Usage](#usage)
5. [Design and Architecture](#design-and-architecture)
6. [Author](#author)

---

## Project Overview

This project utilizes the Robotic Operating System (ROS) and the turtlesim simulator to showcase meaningful movement 
and coordination of one or more robots. The project includes teleoperation, autonomous navigation, collision avoidance, 
and multi-robot behaviours for efficient space coverage.

---

## Features

- **Teleoperation Mode**: Control the robot using the keyboard, with adjustable movement speed.
- **Autonomous Navigation**: Navigate to a specified coordinate within the Turtlesim window.
- **Collision Avoidance**: Override movements to prevent collisions with walls.
- **Vacuum Cleaning Behavior**: Efficiently cover the entire window area with a single robot.
- **Multi-Robot Coordination**: Manage multiple turtles (2 or more) working together for vacuum cleaning.

---

## Setup Instructions

1. Clone the Repository:
   ```bash
   git clone <repository_url>
   cd <repository_name>
   ```

2. Copy the Package to Your Workspace:
   ```bash
   cp -r <repository_name> ~/catkin_ws/src/
   cd ~/catkin_ws
   ```

3. Build the Workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

4. Verify Installation:
   ```bash
   rospack find <package_name>
   ```

---

## Usage

1. **Start ROS Master**:
   ```bash
   roscore
   ```

2. **Launch the Turtlesim Node**:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

3. **Run the Project Scripts**:

   - **Teleoperation**:
     ```bash
     rosrun teleop teleoperation.py
     ```

   - **Autonomous Navigation**:
     ```bash
     rosrun turtle_nav autonomous_navigation.py
     ```

   - **Wall Avoidance**:
     ```bash
     rosrun turtle_wall_avoidance wall_avoidance.py
     ```

   - **Vacuum Cleaning**:
     ```bash
     rosrun vacuum_clean vacuum_turtle.py
     ```

   - **Multi-Turtle Cleaning**:
     ```bash
     rosrun multi_turtle multi_turtle.py
     ```

---

## Design and Architecture

### Nodes and Topics

- **Nodes**: 
  - `teleoperation.py`: Handles keyboard control.
  - `navigate_turtle.py`: Navigates to specified coordinates.
  - `wall_turtle.py`: Stops the turtle movement when wall detected.
  - `vacuum.py`: Implements vacuum cleaning for a single turtle.
  - `vacuum_turtles.py`: Coordinates multiple turtles.

- **Topics**:
  - `/turtle1/cmd_vel`: Publishes velocity commands.
  - `/turtle1/pose`: Subscribes to the turtle's position for navigation and collision avoidance.
  - `/spawn`: Service call to spawn additional turtles.

### Algorithm Flow

1. **Input Handling**:
   - Teleoperation for keyboard-based control.
   - Coordinates for autonomous navigation.

2. **Collision Avoidance**:
   - Monitors the turtle's position and overrides movement commands to avoid collisions.

3. **Multi-Turtle Coordination**:
   - Spawns additional turtles and coordinates their movements to cover the area efficiently.

4. **Vacuum Cleaning**:
   - Implements grid or spiral patterns to maximize area coverage.

### RQT Graph
Included all RQT graphs here to visualize nodes, topics, and their interactions.

1. Task 1 - Teleoperation
2. Task 2 - Autonomous navigation
3. Task 3 - Avoiding Wall Collision
4. Task 4 - Vacuum Cleaning Behavior
5. Task 5 - Multiple Turtle Vacuum
---

## Author

Created as part of PDE4430 Mobile Robotics - Middlesex University Dubai coursework.  
**Email**: zainabkazi.zk@gmail.com
