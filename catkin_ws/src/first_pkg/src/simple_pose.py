#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

class SimplePose:
    def __init__(self):
        """Initializes the class member variables
        """
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_tolerance = 0.02

        self.max_vel = 0.22
        self.max_omega = 2.84

        self.k_rho = 0.3
        self.k_alpha = 0.8

        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Wait for a message on the odom topic to make sure the simulation is started
        rospy.wait_for_message("/odom", Odometry, 10)

    def odom_callback(self, msg):
        """Callback function for the self.sub ROS subscriber

        Args:
            msg (Odometry): The odometry of the robot.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # ROS uses quaternions to describe the roll, pitch, and yaw, thus we must convert it to euler angles
        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def go_to(self, goal_point):
        """Calculates the current distance (rho) to the goal_point as well as the angle to the goal.
        Based on the delta angle to the goal, this function adjusts the linear and angular velocity using proportional control.

        Args:
            goal_point (Point): The goal position.
        """
        speed = Twist()
        rho = 999999999999999999

        while rho > self.goal_tolerance:
            delta_x = goal_point.x - self.x
            delta_y = goal_point.y - self.y
            rho = sqrt(delta_x**2 + delta_y**2)
            rospy.loginfo_throttle_identical(1, "Distance to goal= {}m".format(rho))
            angle_to_goal = atan2(delta_y, delta_x)

            # Calculate the angle difference (alpha)
            alpha = angle_to_goal - self.theta

            # Proportional control for linear and angular velocity
            v = self.k_rho * rho
            omega = self.k_alpha * alpha

            # Limit linear and angular velocity to maximum values
            speed.linear.x = min(v, self.max_vel)
            speed.angular.z = min(omega, self.max_omega)

            # Publish velocity commands
            self.pub.publish(speed)
            rospy.sleep(0.01)

    def stop_robot(self):
        """This function stops the robot
        """
        speed = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub.publish(speed)

if __name__ == '__main__':
    rospy.init_node("speed_controller")
    simple_pose_mover = SimplePose()

    goal = Point()
    goal.x = 3
    goal.y = 4

    try:
        simple_pose_mover.go_to(goal)
    except (KeyboardInterrupt, rospy.ROSException) as e:
        rospy.logerr(e)
    finally:
        simple_pose_mover.stop_robot()
        position_error = sqrt((goal.x - simple_pose_mover.x)**2 + (goal.y - simple_pose_mover.y)**2)
        rospy.loginfo("Final position error: {}m".format(position_error))
