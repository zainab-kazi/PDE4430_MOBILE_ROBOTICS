#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
import math
import random

class MultiTurtleCleaner:
    def __init__(self, num_turtles=2):
        rospy.init_node('multi_turtle_cleaning', anonymous=True)
        
        self.turtles = ['turtle1']
        self.pose_subscribers = {}
        self.pose_data = {}
        self.velocity_publishers = {}

        # Spawn additional turtles
        for i in range(2, num_turtles + 1):
            turtle_name = f"turtle{i}"
            self.spawn_turtle(turtle_name)
            self.turtles.append(turtle_name)

        # Initialize pose subscribers and velocity publishers
        for turtle in self.turtles:
            self.pose_data[turtle] = Pose()
            self.velocity_publishers[turtle] = rospy.Publisher(f'/{turtle}/cmd_vel', Twist, queue_size=10)
            self.pose_subscribers[turtle] = rospy.Subscriber(f'/{turtle}/pose', Pose, self.update_pose, turtle)

        rospy.loginfo("Multi-turtle vacuum cleaning initialized with %d turtles.", num_turtles)

    def spawn_turtle(self, name):
        """Spawns a new turtle at a random location."""
        rospy.wait_for_service('/spawn')
        try:
            spawn_service = rospy.ServiceProxy('/spawn', Spawn)
            x, y = random.uniform(1.0, 10.0), random.uniform(1.0, 10.0)
            spawn_service(x, y, 0, name)
            rospy.loginfo("Spawned turtle: %s at (%.2f, %.2f)", name, x, y)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def update_pose(self, data, turtle_name):
        """Updates the pose data for a turtle."""
        self.pose_data[turtle_name] = data

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def move_turtle(self, turtle, speed=2.0, angular_speed=1.0):
        """Moves a turtle in a random direction."""
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = angular_speed * (1 if random.choice([True, False]) else -1)
        self.velocity_publishers[turtle].publish(vel_msg)

    def vacuum_clean(self):
        """Main cleaning behavior."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for turtle in self.turtles:
                pose = self.pose_data[turtle]
                # If the turtle is near a wall, change direction
                if pose.x < 1.0 or pose.x > 10.0 or pose.y < 1.0 or pose.y > 10.0:
                    self.move_turtle(turtle, speed=2.0, angular_speed=2.0)
                else:
                    self.move_turtle(turtle)
            rate.sleep()

if __name__ == "__main__":
    try:
        cleaner = MultiTurtleCleaner(num_turtles=3)  # Adjust number of turtles here
        cleaner.vacuum_clean()
    except rospy.ROSInterruptException:
        pass
