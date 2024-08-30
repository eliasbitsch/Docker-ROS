#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):                                    # Define a function called 'callback' that receives a parameter
                                                      # named 'msg'
    string_with_points = msg.data
    rospy.loginfo("{} Dumbledore shouted".format(string_with_points)) # Print the 'msg'

rospy.init_node('counter_subscriber')                   # Initiate a Node called 'topic_subscriber'

sub = rospy.Subscriber('/points', String, callback)   # Create a Subscriber object that will listen to the /points
                                                      # topic and will cal the 'callback' function each time it receives
                                                      # a String from the topic
rospy.spin()                                          # Create a loop that will keep th