#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

class Distance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('distance', anonymous=True)

        # Subscribe to the positions of turtle1 and turtle2
        self.turtle1_pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose_callback)
        self.turtle2_pose_sub = rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_pose_callback)

        # Publisher for the relative distance
        self.distance_pub = rospy.Publisher('/relative_distance', Float32, queue_size=10)

        # Variables to store the positions of the turtles
        self.turtle1_position = None
        self.turtle2_position = None

    def turtle1_pose_callback(self, msg):
        """Callback function to store turtle1's position."""
        self.turtle1_position = (msg.x, msg.y)
        self.calculate_and_publish_distance()

    def turtle2_pose_callback(self, msg):
        """Callback function to store turtle2's position."""
        self.turtle2_position = (msg.x, msg.y)
        self.calculate_and_publish_distance()

    def calculate_and_publish_distance(self):
        """Calculate and publish the relative distance between turtle1 and turtle2."""
        if self.turtle1_position and self.turtle2_position:
            x1, y1 = self.turtle1_position
            x2, y2 = self.turtle2_position

            # Calculate the Euclidean distance between the two turtles
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            # Create a Float32 message to publish the distance
            distance_msg = Float32()
            distance_msg.data = distance

            # Publish the distance
            self.distance_pub.publish(distance_msg)

            #rospy.loginfo(f"Distance between turtle1 and turtle2: {distance:.2f} meters")

if __name__ == '__main__':
    try:
        # Create an instance of the Distance class
        distance_node = Distance()

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Distance calculator node shut down.")

