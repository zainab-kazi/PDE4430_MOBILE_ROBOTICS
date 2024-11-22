#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_callback(data, anti_collision_publisher):
    """
    Callback function to stop the turtle if it approaches the wall.
    :param data: The pose data of the turtle.
    :param anti_collision_publisher: Publisher to control the turtle's velocity.
    """
    # Create a Twist message for publishing
    twist_cmd = Twist()

    # Check if the turtle is too close to the walls
    if data.x > 10.5 or data.x < 0.5 or data.y > 10.5 or data.y < 0.5:
        #rospy.loginfo("Imminent Turtle collision at: x=%f, y=%f, theta=%f", data.x, data.y, data.theta)
        
        # Stop the turtle
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        anti_collision_publisher.publish(twist_cmd)
    else:
        # Allow the turtle to keep moving
        twist_cmd.linear.x = 1.0
        twist_cmd.angular.z = 0.0
        anti_collision_publisher.publish(twist_cmd)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('anticollision', anonymous=True)
        
        # Publisher to control the turtle's velocity
        turtle1_anti_collision = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to monitor the turtle's pose
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback, turtle1_anti_collision)
        
        # Maintain loop rate
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Turtle anti-collision node started.")
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        rospy.logerr("Execution interrupted! The turtle may crash.")
