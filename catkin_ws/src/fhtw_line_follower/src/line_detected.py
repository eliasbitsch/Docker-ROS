#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class LineDetectedBoolNode:
    def __init__(self):
        rospy.init_node('line_detected_bool_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.line_detected_pub = rospy.Publisher('/line_detected_bool', Bool, queue_size=10)

        # Initialize HSV threshold values for yellow
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Convert BGR image to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Generate binary image
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]

        # Initialize line detection status
        line_detected = False

        if contours:
            # Get the bounding box of the largest contour
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            # Draw the rectangle at the lower third of the image
            if cv2.contourArea(contour) > 0:
                line_detected = True

        # Publish the line_detected status
        self.line_detected_pub.publish(Bool(line_detected))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LineDetectedBoolNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
