#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import time

class Distance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('distance', anonymous=True)

        # Subscribe to the positions of turtle1 and turtle2
        self.turtle1_pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose_callback)
        self.turtle2_pose_sub = rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_pose_callback)

        # Publisher for the relative distance
        self.distance_pub = rospy.Publisher('/relative_distance', Float32, queue_size=10)
        
        #Publishers for stopping turtles (cmd_vel)
        self.turtle1_cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        self.turtle2_cmd_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size = 10)

        # Variables to store the positions of the turtles
        self.turtle1_position = None
        self.turtle2_position = None
        
        #Define a "too close" threshold
        self.distance_threshold = 1.5 
        
        #Initialize a timer for periodic stop command
        self.stop_timer = rospy.Timer(rospy.Duration(0.01), self.check_distances)
        
        #Flag to track if the turtles are too close
        self.turtles_should_stop = False
        
        #Remember the stopping time in order not to continuously block the turtles
        self.stopping_time = None
        
        #Remember the previous distance to compute if turtle are getting closer or not 
        self.previous_distance = None 
        self.previous_x1 = None
        self.previous_y1 = None 
        self.previous_x2 = None
        self.previous_y2 = None
        

    def turtle1_pose_callback(self, msg):
        """Callback function to store turtle1's position."""
        self.turtle1_position = (msg.x, msg.y)

    def turtle2_pose_callback(self, msg):
        """Callback function to store turtle2's position."""
        self.turtle2_position = (msg.x, msg.y)	    		
	    		
	    	
    def check_distances(self, event):
    	if self.turtle1_position and self.turtle2_position:
    		x1, y1 = self.turtle1_position
    		x2, y2 = self.turtle2_position 
    		
    		#Calculate the Euclidean distance between the two turtles 
    		distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    		
    		#Publish the distance
    		distance_msg = Float32()
    		distance_msg.data = distance
    		self.distance_pub.publish(distance_msg)
    		
    		#check if turtles are too close 
    		if not self.turtles_should_stop and distance < self.distance_threshold and distance < self.previous_distance :
    			self.turtles_should_stop = True 
    			self.stopping_time = time.time()
    			rospy.logwarn(f"Turtles are too close! Distance: {distance:.2f} m. Stopping turtles. ")
    			
    		#check if turtles are going out of the boundaries
    		x1_out = (x1 > 10 and x1 > self.previous_x1) or (x1 < 1 and x1 < self.previous_x1) 
    		x2_out = (x2 > 10 and x2 > self.previous_x2) or (x2 < 1 and x2 < self.previous_x2) 
    		y1_out = (y1 > 10 and y1 > self.previous_y1) or (y1 < 1 and y1 < self.previous_y1) 
    		y2_out = (y2 > 10 and y2 > self.previous_y2) or (y2 < 1 and y2 < self.previous_y2) 
    		out_boundaries = x1_out or x2_out or y1_out or y2_out 
    		
    		if not self.turtles_should_stop and out_boundaries : 
    			self.turtles_should_stop = True
    			self.stopping_time = time.time()
    			rospy.logwarn(f"Turtles are getting out! Stopping turtles. ")
    			
    		if self.turtles_should_stop : 
    			if time.time() - self.stopping_time > 1.0:
    				self.turtles_should_stop = False
    				
    			else: 
    				stop_msg = Twist()
    				stop_msg.linear.x = 0
    				stop_msg.angular.z = 0
    				self.turtle1_cmd_vel_pub.publish(stop_msg)
    				self.turtle2_cmd_vel_pub.publish(stop_msg)
    				#rospy.loginfo("Sent stop command to both turtles")
	    		
    		self.previous_distance = distance
    		self.previous_x1 = x1
    		self.previous_y1 = y1
    		self.previous_x2 = x2
    		self.previous_y2 = y2

if __name__ == '__main__':
    try:
        # Create an instance of the Distance class
        distance_node = Distance()

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Distance calculator node shut down.")

