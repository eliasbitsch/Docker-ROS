#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

class MotionNode:
    def __init__(self):
        rospy.init_node('motion_node', anonymous=True)

        self.line_pos_sub = rospy.Subscriber('/line_position', Int32, self.line_position_callback)
        self.line_x_pos_sub = rospy.Subscriber('/line_x_position', Float32, self.line_x_position_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define speeds
        self.forward_speed = 0.3  # Linear speed
        self.angular_speed = 1.0  # Angular speed for turning in bang-bang controller

        # Proportional control gains
        self.kp = 0.01  # Proportional gain for turning in proportional controller

        self.frame_width = 640  # Default frame width (should match your camera resolution)

        # Store the latest x position and line position
        self.latest_x_pos = None
        self.latest_line_pos = None

        # Set the default steering method and validate
        self.steering_method = rospy.get_param('~steering_method', 'proportional')
        if self.steering_method not in ['proportional', 'bang_bang']:
            rospy.logwarn("Invalid steering method parameter, defaulting to 'proportional'")
            self.steering_method = 'proportional'
        rospy.loginfo("Initial steering method set to: {}".format(self.steering_method))

        # Print usage information
        rospy.loginfo("Usage: rosparam set /motion_node/steering_method bang_bang or rosparam set /motion_node/steering_method proportional")

        # Create a timer to periodically check for parameter updates
        self.param_update_timer = rospy.Timer(rospy.Duration(1), self.update_params)

    def update_params(self, event):
        # Update the steering method parameter
        new_steering_method = rospy.get_param('~steering_method', 'proportional')
        if new_steering_method != self.steering_method:
            if new_steering_method in ['proportional', 'bang_bang']:
                self.steering_method = new_steering_method
                rospy.loginfo("Updated steering method to: {}".format(self.steering_method))
            else:
                rospy.logwarn("Invalid steering method parameter, cannot update to: {}".format(new_steering_method))

    def line_position_callback(self, msg):
        self.latest_line_pos = msg.data
        self.control_robot()

    def line_x_position_callback(self, msg):
        self.latest_x_pos = msg.data
        self.control_robot()

    def control_robot(self):
        if self.steering_method == 'bang_bang':
            self.bang_bang_control()
        elif self.steering_method == 'proportional':
            self.proportional_control()
        else:
            rospy.logwarn("Unknown steering method: {}".format(self.steering_method))

    def bang_bang_control(self):
        if self.latest_line_pos is None:
            return

        twist_msg = Twist()
        if self.latest_line_pos == 0:  # Line is to the left
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.angular_speed  # Turn right
        elif self.latest_line_pos == 2:  # Line is to the right
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = -self.angular_speed  # Turn left
        else:  # Line is in the center
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = 0  # Go straight

        self.cmd_vel_pub.publish(twist_msg)

    def proportional_control(self):
        if self.latest_x_pos is None:
            return

        twist_msg = Twist()
        error = self.latest_x_pos - self.frame_width // 2
        twist_msg.linear.x = self.forward_speed
        twist_msg.angular.z = -self.kp * error  # Proportional control

        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MotionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
