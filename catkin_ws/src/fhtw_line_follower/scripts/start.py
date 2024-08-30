#!/usr/bin/python3

import subprocess
import time
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

# Global variables for process management
goal_reached_triggered = False
follow_line_process = None
line_detected_process = None

def is_roscore_running():
    try:
        subprocess.check_output(["rosnode", "list"])
        return True
    except subprocess.CalledProcessError:
        return False

def start_roslaunch(package, launch_file):
    # Start a ROS launch file using subprocess
    return subprocess.Popen(['roslaunch', package, launch_file])

def publish_goal(x, y, z, qx, qy, qz, qw):
    # Publisher to send a goal position to the move_base node
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    if not rospy.core.is_initialized():
        rospy.init_node('goal_publisher', anonymous=True)
        time.sleep(1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    if not rospy.is_shutdown():
        pub.publish(goal)
        rospy.loginfo(f"Published goal position to /move_base_simple/goal: ({x}, {y})")
    else:
        rospy.logwarn("Node is shutting down. Goal not published.")

def goal_reached_callback(data):
    global goal_reached_triggered, follow_line_process
    for status in data.status_list:
        if status.status == 3 and not goal_reached_triggered:
            # Goal reached status is indicated by status 3
            goal_reached_triggered = True
            print("Goal reached!")
            if follow_line_process is None:
                # Start follow_line.launch process if not already running
                follow_line_process = subprocess.Popen(['roslaunch', 'fhtw_line_follower', 'follow_line.launch'])

def line_detected_callback(msg):
    global follow_line_process, goal_reached_triggered
    if not msg.data:
        if follow_line_process:
            # If line is not detected, terminate follow_line.launch process
            print("Line not detected. Terminating follow_line.launch process.")
            follow_line_process.terminate()
            follow_line_process.wait()
            follow_line_process = None
            goal_reached_triggered = False
            if not rospy.is_shutdown():
                # Publish a new goal if the node is not shutting down
                publish_goal(7.0, 0.5, 0.0, 0.0, 0.0, -0.707, 0.707)

def monitor_goal():
    # Subscribe to topics to monitor goal status and line detection
    rospy.Subscriber('/move_base/status', GoalStatusArray, goal_reached_callback)
    rospy.Subscriber('/line_detected_bool', Bool, line_detected_callback)
    rospy.spin()

def cleanup_processes(initial_launch_process):
    global follow_line_process, line_detected_process
    print("Cleaning up...")
    if follow_line_process:
        follow_line_process.terminate()
        follow_line_process.wait()
    if line_detected_process:
        line_detected_process.terminate()
        line_detected_process.wait()
    if initial_launch_process:
        initial_launch_process.terminate()
        initial_launch_process.wait()
    print("All processes terminated.")

def main():
    global goal_reached_triggered, follow_line_process, line_detected_process
    goal_reached_triggered = False
    follow_line_process = None
    line_detected_process = None
    initial_launch_process = None

    try:
        # Start the initial ROS launch file
        initial_launch_process = start_roslaunch('fhtw_line_follower', 'fhtw_line_follower_project.launch')
        time.sleep(10)  # Wait for Gazebo to fully load

        # Start the line_detected.launch file
        line_detected_process = start_roslaunch('fhtw_line_follower', 'line_detected.launch')

        # Publish the initial goal position
        publish_goal(7.0, 0.5, 0.0, 0.0, 0.0, -0.707, 0.707)

        # Monitor the goal status
        monitor_goal()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_processes(initial_launch_process)

if __name__ == '__main__':
    main()
