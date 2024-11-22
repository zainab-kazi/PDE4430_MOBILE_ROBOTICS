#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Global variables
velocity_publisher = None
turtlesim_pose = Pose()

# Window boundaries
x_min, y_min = 0.0, 0.0
x_max, y_max = 11.0, 11.0
PI = 3.14159265359

def move(speed, distance, is_forward):
    """Moves the turtle in a straight line."""
    vel_msg = Twist()
    vel_msg.linear.x = abs(speed) if is_forward else -abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0.0
    rate = rospy.Rate(100)

    while current_distance < distance:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed * (t1 - t0)
        rate.sleep()

    # Stop the turtle after moving
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def rotate(angular_speed, angle, clockwise):
    """Rotates the turtle."""
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -abs(angular_speed) if clockwise else abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0
    rate = rospy.Rate(100)

    while current_angle < angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()

    # Stop the turtle after rotation
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def degrees_to_radians(angle_in_degrees):
    """Converts degrees to radians."""
    return angle_in_degrees * PI / 180.0

def set_desired_orientation(desired_angle_radians):
    """Sets the turtle to the desired absolute orientation."""
    relative_angle_radians = desired_angle_radians - turtlesim_pose.theta
    clockwise = relative_angle_radians < 0
    rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise)

def pose_callback(pose_message):
    """Callback function to update the turtle's pose."""
    global turtlesim_pose
    turtlesim_pose = pose_message

def get_distance(x1, y1, x2, y2):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def move_to_goal(goal_pose, distance_tolerance):
    """Moves the turtle to a goal position."""
    rate = rospy.Rate(10)
    vel_msg = Twist()

    while get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance:
        # Linear velocity
        vel_msg.linear.x = 1.5 * get_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4 * (math.atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta)

        velocity_publisher.publish(vel_msg)
        rate.sleep()

    # Stop the turtle after reaching the goal
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def grid_clean():
    """Performs grid cleaning."""
    goal_pose = Pose()
    goal_pose.x = 1
    goal_pose.y = 1
    move_to_goal(goal_pose, 0.01)
    set_desired_orientation(0)

    move(2, 9, True)
    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    move(2, 9, True)

    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    move(2, 1, True)
    rotate(degrees_to_radians(10), degrees_to_radians(90), False)
    move(2, 9, True)

    rotate(degrees_to_radians(30), degrees_to_radians(90), True)
    move(2, 1, True)
    rotate(degrees_to_radians(30), degrees_to_radians(90), True)
    move(2, 9, True)

def spiral_clean():
    """Performs spiral cleaning."""
    vel_msg = Twist()
    rk = 0.5
    constant_speed = 4

    rate = rospy.Rate(1)
    while turtlesim_pose.x < x_max and turtlesim_pose.y < y_max:
        rk += 0.5
        vel_msg.linear.x = rk
        vel_msg.angular.z = constant_speed
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('robot_cleaner', anonymous=True)

        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        rospy.loginfo("Robot cleaner node started")
        spiral_clean()  # You can also call grid_clean()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
