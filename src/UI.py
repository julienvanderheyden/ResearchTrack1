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
        self.x = rospy.get_param('~x', 5.0)
        self.y = rospy.get_param('~y', 5.0)
        self.theta = rospy.get_param('~theta', 0.0)
        self.name = rospy.get_param('~name', 'turtle2')

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
            user_input = input("Enter some text: ")  # Wait for the user to enter text
            self.input_callback(user_input)

    def input_callback(self, text):
    	"""Callback function that checks user input and logs the details."""
    	# Try to parse the input and validate the format
    	try:
    		# Split the input text by spaces
    		parts = text.split()
		
    		# Ensure the input has exactly 3 parts: turtle number, linear velocity, angular velocity
    		if len(parts) != 3:
    			raise ValueError("Incorrect format! Expected 'turtle_number linear_velocity angular_velocity'.")
		
    		# Parse the first part as turtle number (either 1 or 2)
    		turtle_number = parts[0]
    		if turtle_number not in ['1', '2']:
    			raise ValueError("Turtle number must be either '1' or '2'.")
		
    		# Check if the linear velocity is a valid number (float)
    		try:
    			linear_velocity = float(parts[1])  # Try to convert linear velocity to float
    		except ValueError:
    			raise ValueError("Linear velocity must be a valid number.")
		
    		# Parse the third part as angular velocity (float)
    		try:
    			angular_velocity = float(parts[2])
    		except ValueError:
    			raise ValueError("Angular velocity must be a valid number.")
		
    		# Log the validated input
    		rospy.loginfo(f"Control command received: Turtle {turtle_number} | Linear Velocity: {linear_velocity} | Angular Velocity: {angular_velocity}")
    		
    		# Start sending the velocities for one minute
    		self.send_velocity(turtle_number, linear_velocity, angular_velocity)
            
		
    	except ValueError as e:
    		# If an error occurs, log an error message
    		rospy.logerr(f"Invalid input: {e}")
    		
    def send_velocity(self, turtle_number, linear_velocity, angular_velocity):
    	"""Sends the linear and angular velocities for one minute to the selected turtle."""
    	# Create a Twist message
    	cmd_vel = Twist()
    	cmd_vel.linear.x = linear_velocity
    	cmd_vel.angular.z = angular_velocity
        
    	# Determine the correct publisher based on the turtle number
    	if turtle_number == '1':
    		pub = self.cmd_vel_pub_turtle1
    	else:
    		pub = self.cmd_vel_pub_turtle2

    	# Publish the velocity for 1 minute
    	start_time = time.time()
    	while time.time() - start_time < 1.0 and not rospy.is_shutdown():
    		pub.publish(cmd_vel)
    		rospy.sleep(0.1)  # Sleep for a short period to avoid overloading the system

    	cmd_vel.linear.x = 0
    	cmd_vel.angular.z = 0
    	pub.publish(cmd_vel)
    	rospy.loginfo(f"Stopped publishing velocities to turtle {turtle_number} after 1 second.")



if __name__ == '__main__':
    try:
        node = UI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("UI node shut down.")

