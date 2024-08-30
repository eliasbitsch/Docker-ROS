#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32, Int32, Bool



class LineFollowerImageProcessorNode:
    def __init__(self):
        rospy.init_node('detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        self.image_pub = rospy.Publisher('/line_detection', Image, queue_size=10) # Publisher for line_detection
        self.x_pos_pub = rospy.Publisher('/line_x_position', Float32, queue_size=10) # Publisher for line_x_position
        self.line_pos_pub = rospy.Publisher('/line_position', Int32, queue_size=10) # Publisher for line_position

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

        # Draw a rectangle at the lower third of the image
        if contours:
            # Get the bounding box of the largest contour
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            # Draw the rectangle
            cv2.rectangle(frame, (x, y + int(0.6 * h)), (x + w, y + h), (0, 0, 255), 2)

            # Calculate the centroid of the contour
            if cv2.contourArea(contour) > 0:
                M = cv2.moments(contour)
                cx = int(M["m10"] / M["m00"])
                cy = int((y + int(0.6 * h) + y + h) / 2)  # Calculate centroid y-coordinate
                # Draw the green dot at the middle of the width of the contour
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Publish the x-position of the center of the line
                self.x_pos_pub.publish(Float32(cx))

                # Determine the line position (left, center, right)
                frame_width = frame.shape[1]
                if cx < frame_width // 3:
                    line_position = 0  # Left
                elif cx > 2 * frame_width // 3:
                    line_position = 2  # Right
                else:
                    line_position = 1  # Center

                # Publish the line position
                self.line_pos_pub.publish(Int32(line_position))

                # Set line_detected to True if line is detected
                line_detected = True

        # Publish the modified image
        modified_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(modified_image_msg)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LineFollowerImageProcessorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
