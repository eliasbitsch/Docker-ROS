#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from numpy import asarray

closest_points = []  # Declare closest_points array 

# Callback odometry messages
def odometry_callback(odom_msg):
    rospy.logdebug("Received Odometry Message:\n%s", odom_msg)
    rate.sleep()

# Callback laser scan messages
def laser_scan_callback(scan_msg):
    global closest_points
    # Extract the 5 closest points
    closest_points = sorted(scan_msg.ranges)[:5]

# Initialize ROS node
rospy.init_node('unit2_hw', log_level=rospy.DEBUG)

# publisher for the 'n_smallest' topic
n_smallest_publisher = rospy.Publisher('n_smallest', Float32MultiArray, queue_size=10)

# loop rate 2 Hz
rate = rospy.Rate(2)

# Main loop continues until the ROS node is shut down
while not rospy.is_shutdown():
    # Create subscribers for odometry and laser scan topics
    sub_odom = rospy.Subscriber('/odom', Odometry, odometry_callback)
    sub_scan = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)

    # Check if closest_points array is not empty
    if closest_points:
        # Convert closest_points to a NumPy array and then to a Float32MultiArray
        array = asarray(closest_points)
        closest_points_array = Float32MultiArray(data=array)
        
        # Publish the closest_points_array to the 'n_smallest' topic
        n_smallest_publisher.publish(closest_points_array)

    # Sleep to maintain the loop rate
    rate.sleep()