#!/usr/bin/env python3

import rospy
import threading
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import time

class UI:
    def __init__(self):
        # Initialize the node
        rospy.init_node('UI', anonymous=True)

        # Read parameters for the turtle's position, orientation, and name
        self.x = rospy.get_param('~x', 7.0)
        self.y = rospy.get_param('~y', 7.0)
        self.theta = rospy.get_param('~theta', 0.0)
        self.name = 'turtle2'

        # Wait for the '/spawn' service to be available
        rospy.loginfo("Waiting for the turtlesim 'spawn' service...")
        rospy.wait_for_service('/spawn')

        try:
            # Create a proxy for the '/spawn' service
            self.spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to connect to '/spawn' service: {e}")
            rospy.signal_shutdown("Service error")

        # Spawn the turtle
        self.spawn_turtle()
        
        # Publisher for cmd_vel
        self.cmd_vel_pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        
        self.turtle_number = None
        self.linear_velocity = None
        self.angular_velocity = None 
        self.turtle_moving = False
        self.moving_start_time = None
        
        #Initialize a timer for periodic control command
        self.moving_timer = rospy.Timer(rospy.Duration(0.2), self.send_velocity)

        # Start listening to the terminal in a separate thread
        self.listen_to_input()
        


    def spawn_turtle(self):
        """Spawns a new turtle in the turtlesim environment."""
        try:
            rospy.loginfo(f"Spawning turtle '{self.name}' at x: {self.x}, y: {self.y}, theta: {self.theta}")
            response = self.spawn_service(self.x, self.y, self.theta, self.name)
            rospy.loginfo(f"Turtle '{response.name}' spawned successfully!")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to '/spawn' failed: {e}")

    def listen_to_input(self):
        """Listens to terminal input and calls the callback function with the input text."""
        threading.Thread(target=self.read_input, daemon=True).start()

    def read_input(self):
        """Reads input from the terminal and calls the callback."""
        while not rospy.is_shutdown():
            user_input = input("Enter control line: ")  # Wait for the user to enter text
            self.input_callback(user_input)

    def input_callback(self, text):
    	"""Callback function that checks user input and logs the details."""
    	# Try to parse the input and validate the format
    	try:
    		# Split the input text by spaces
    		parts = text.split()
		
    		# Ensure the input has exactly 3 parts: turtle number, linear velocity, angular velocity
    		if len(parts) != 3:
    			raise ValueError("Incorrect format! Expected 3 arguments which are : 'turtle_number linear_velocity angular_velocity'.")
		
    		# Parse the first part as turtle number (either 1 or 2)
    		self.turtle_number = parts[0]
    		if self.turtle_number not in ['1', '2']:
    			raise ValueError("Turtle number must be either '1' or '2'.")
		
    		# Check if the linear velocity is a valid number (float)
    		try:
    			self.linear_velocity = float(parts[1])  # Try to convert linear velocity to float
    		except ValueError:
    			raise ValueError("Linear velocity must be a valid number.")
		
    		# Parse the third part as angular velocity (float)
    		try:
    			self.angular_velocity = float(parts[2])
    		except ValueError:
    			raise ValueError("Angular velocity must be a valid number.")
    			
    		#Apply thresholds for both linear and angular velocities
    		MAX_LINEAR_VELOCITY = 2.5
    		MAX_ANGULAR_VELOCITY = 1.5
    		
    		if self.linear_velocity > MAX_LINEAR_VELOCITY:
    			rospy.logwarn(f"linear velocity {self.linear_velocity} exceeds max threshold. Clamped to {MAX_LINEAR_VELOCITY}.")
    			self.linear_velocity = MAX_LINEAR_VELOCITY
    		elif self.linear_velocity < -MAX_LINEAR_VELOCITY:
    			rospy.logwarn(f"linear velocity {self.linear_velocity} below min threshold. Clamped to {-MAX_LINEAR_VELOCITY}.")
    			self.linear_velocity = -MAX_LINEAR_VELOCITY
    		
    		if self.angular_velocity > MAX_ANGULAR_VELOCITY:
    			rospy.logwarn(f"Angular velocity {self.angular_velocity} exceeds max threshold. Clamped to {MAX_ANGULAR_VELOCITY}.")
    			self.angular_velocity = MAX_ANGULAR_VELOCITY
    		elif self.angular_velocity < -MAX_ANGULAR_VELOCITY:
    			rospy.logwarn(f"Angular velocity {self.angular_velocity} below min threshold? Clamped to {-MAX_ANGULAR_VELOCITY}.")
    			self.angular_velocity = -MAX_ANGULAR_VELOCITY
    		
		
    		# Log the validated input
    		rospy.loginfo(f"Control command received: Turtle {self.turtle_number} | Linear Velocity: {self.linear_velocity} | Angular Velocity: {self.angular_velocity}")
    		
    		# Start sending the velocities for one minute
    		self.moving_start_time = time.time()
    		self.turtle_moving = True
            
		
    	except ValueError as e:
    		# If an error occurs, log an error message
    		rospy.logerr(f"Invalid input: {e}")
    		
    def send_velocity(self, event):
    	"""Sends the linear and angular velocities for one minute to the selected turtle."""
    	if self.turtle_moving : 
    		# Create a Twist message
    		cmd_vel = Twist()
    		
    		# Determine the correct publisher based on the turtle number
    		if self.turtle_number == '1':
    			pub = self.cmd_vel_pub_turtle1
    		else:
    			pub = self.cmd_vel_pub_turtle2
    			
    		if time.time() - self.moving_start_time > 1.0:
    			#Stop turtle
    			cmd_vel.linear.x = 0
    			cmd_vel.angular.z = 0
    			pub.publish(cmd_vel)
    			self.turtle_moving = False
    			
    		else : 
    			
    			cmd_vel.linear.x = self.linear_velocity
    			cmd_vel.angular.z = self.angular_velocity
        
    			pub.publish(cmd_vel)




if __name__ == '__main__':
    try:
        node = UI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("UI node shut down.")

