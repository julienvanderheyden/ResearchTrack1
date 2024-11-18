#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

def spawn_turtle():
    # Initialize the ROS node
    rospy.init_node('spawn_turtle_node', anonymous=True)
    
    # Wait for the 'spawn' service to be available
    rospy.loginfo("Waiting for the turtlesim 'spawn' service...")
    rospy.wait_for_service('/spawn')

    try:
        # Create a proxy for the spawn service
        spawn_turtle_service = rospy.ServiceProxy('/spawn', Spawn)
        
        # Parameters for the new turtle
        x = rospy.get_param('~x', 5.0)  # Default X position: 5.0
        y = rospy.get_param('~y', 5.0)  # Default Y position: 5.0
        theta = rospy.get_param('~theta', 0.0)  # Default orientation: 0 radians
        name = rospy.get_param('~name', 'turtle2')  # Default name: turtle2
        
        # Call the service to spawn a new turtle
        rospy.loginfo(f"Spawning turtle '{name}' at x: {x}, y: {y}, theta: {theta}")
        response = spawn_turtle_service(x, y, theta, name)
        rospy.loginfo(f"Turtle '{response.name}' spawned successfully!")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    
if __name__ == '__main__':
    try:
        spawn_turtle()
    except rospy.ROSInterruptException:
        pass

