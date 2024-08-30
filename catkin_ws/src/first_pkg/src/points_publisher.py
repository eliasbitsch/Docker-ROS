#!/usr/bin/python3
import rospy        # Import the Python library for ROS
from std_msgs.msg import String      # Import the String message from the std_msgs package

rospy.init_node('points_publisher')        # Initiate a Node named 'points_publisher'
pub = rospy.Publisher('/points', String, queue_size=1)  # Create a Publisher object, that will publish on the /points topic messages of type Int32

rate = rospy.Rate(2)        # Set a publish rate of 2 Hz
house_points = String()     # Create a variable of type String
counter = 0                 # Create a counter
house = "Gryffindor"

while not rospy.is_shutdown():  # Create a loop that will go until the program is stopped
  house_points.data = "{} points for {}!".format(counter, house)             # Initialize the data part of the variable
  pub.publish(house_points)     # Publish the message house_points to the topic /points
  counter += 1                  # Increase the counter

  rate.sleep()                  # Sleep for the desired rate (2)


  