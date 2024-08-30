#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist, Quaternion, Pose
from math import atan2, sqrt, cos, sin, pi



class SimplePose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_tolerance = 0.05

        self.max_vel = 0.22
        self.max_omega = 2.84

        self.k_rho = 0.3
        self.k_alpha = 0.8

        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_message("/odom", Odometry, 10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def go_to(self, goal_point, goal_theta):
        speed = Twist()
        rho = 999999999999999999

        while rho > self.goal_tolerance:
            delta_x = goal_point.x - self.x
            delta_y = goal_point.y - self.y
            rho = sqrt(delta_x**2 + delta_y**2)
            rospy.loginfo_throttle_identical(1, "Distance to goal = {}m".format(rho))
            angle_to_goal = atan2(delta_y, delta_x)

            alpha = angle_to_goal - self.theta
            v = self.k_rho * rho
            omega = self.k_alpha * alpha

            speed.linear.x = min(v, self.max_vel)
            speed.angular.z = min(omega, self.max_omega)

            self.pub.publish(speed)
            rospy.sleep(0.01)

        self.stop_robot()

        final_orientation = quaternion_from_euler(0, 0, goal_theta)
        final_orientation_msg = Quaternion(*final_orientation)
        final_goal_pose = Pose(Point(goal_point.x, goal_point.y, 0.0), final_orientation_msg)
        self.adjust_orientation(final_goal_pose)

    def adjust_orientation(self, final_goal_pose):
    # Extract the quaternion from the pose
        final_goal_quat = final_goal_pose.orientation

    # Convert the quaternion to Euler angles
        _, _, final_goal_theta = euler_from_quaternion([final_goal_quat.x, final_goal_quat.y, final_goal_quat.z, final_goal_quat.w])

        while abs(self.theta - final_goal_theta) > self.goal_tolerance:
        # Calculate the angular difference (alpha)
            alpha = final_goal_theta - self.theta

        # Ensure that alpha is within the range of -pi to pi
            alpha = atan2(sin(alpha), cos(alpha))

        # Proportional control for angular velocity
            omega = self.k_alpha * alpha

        # Limit angular velocity to the maximum value
            speed = Twist()
            speed.angular.z = min(omega, self.max_omega)

        # Publish velocity commands
            self.pub.publish(speed)
            rospy.sleep(0.01)

    # Stop the robot after adjusting the orientation
        self.stop_robot()

    def stop_robot(self):
        speed = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub.publish(speed)

if __name__ == '__main__':
    rospy.init_node("speed_controller")
    simple_pose_mover = SimplePose()

    
    goal = Point()
    goal.x = 0
    goal.y = 0
    
    goal_theta = -pi/2

    try:
        simple_pose_mover.go_to(goal, goal_theta)
    except (KeyboardInterrupt, rospy.ROSException) as e:
        rospy.logerr(e)
    finally:
        position_error = sqrt((goal.x - simple_pose_mover.x)**2 + (goal.y - simple_pose_mover.y)**2)
        rospy.loginfo("Final position error: {}m".format(position_error))