#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class Teleoperation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('teleoperation', anonymous=True)
        
        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Initialize the velocity message
        self.twist = Twist()
        self.speed = 1.0  # Default speed
        
        # Key bindings for movement
        self.key_bindings = {
            'w': (1, 0),  # Forward
            's': (-1, 0),  # Backward
            'a': (0, 1),  # Rotate left
            'd': (0, -1),  # Rotate right
        }
        
        # Listen to keyboard events
        print("Use 'W', 'A', 'S', 'D' to move the turtle. Press '+' to increase speed and '-' to decrease speed. Press 'ESC' to quit.")
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        """Callback for key press events."""
        try:
            if key.char in self.key_bindings:
                linear, angular = self.key_bindings[key.char]
                self.twist.linear.x = linear * self.speed
                self.twist.angular.z = angular * self.speed
                self.vel_pub.publish(self.twist)
            
            elif key.char == '+':  # Increase speed
                self.speed += 0.5
                print(f"Speed increased to {self.speed}")
            
            elif key.char == '-':  # Decrease speed
                self.speed = max(0.5, self.speed - 0.5)
                print(f"Speed decreased to {self.speed}")
        
        except AttributeError:
            # Handle special keys
            if key == keyboard.Key.esc:
                rospy.loginfo("Exiting teleoperation...")
                self.listener.stop()
                rospy.signal_shutdown("User exited.")
                return

    def stop(self):
        """Stops the turtle's movement."""
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)

if __name__ == "__main__":
    try:
        teleop = Teleoperation()
        rospy.spin()
    except rospy.ROSInterruptException:
        teleop.stop()
