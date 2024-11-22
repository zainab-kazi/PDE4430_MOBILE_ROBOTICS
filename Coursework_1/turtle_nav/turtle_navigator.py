#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavigator:
    def __init__(self):
        rospy.init_node('turtle_autonomous_navigation', anonymous=True)
        
        # Publisher and Subscriber
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10 Hz

    def update_pose(self, data):
        """Callback function to update the turtle's current pose."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Calculate the Euclidean distance to the goal."""
        return math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) +
                         math.pow((goal_pose.y - self.pose.y), 2))

    def linear_velocity(self, goal_pose, constant=1.5):
        """Calculate the linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def angular_velocity(self, goal_pose, constant=6):
        """Calculate the angular velocity."""
        return constant * (math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

    def move_to_goal(self, x_goal, y_goal):
        """Move the turtle to the goal coordinates."""
        goal_pose = Pose()
        goal_pose.x = x_goal
        goal_pose.y = y_goal

        distance_tolerance = 0.01  # Stop within 1 cm of the target
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Calculate velocities
            vel_msg.linear.x = self.linear_velocity(goal_pose)
            vel_msg.angular.z = self.angular_velocity(goal_pose)

            # Publish velocities
            self.velocity_publisher.publish(vel_msg)

            # Maintain control frequency
            self.rate.sleep()

        # Stop the turtle after reaching the goal
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Goal Reached!")

if __name__ == '__main__':
    try:
        navigator = TurtleNavigator()
        x_goal = float(input("Enter x-coordinate of the goal: "))
        y_goal = float(input("Enter y-coordinate of the goal: "))
        navigator.move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
