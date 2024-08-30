#!/usr/bin/python3

import rospy
import time

# Options: DEBUG, INFO, WARN, ERROR, FATAL
rospy.init_node('logger_test', log_level=rospy.DEBUG)
rate = rospy.Rate(0.5)

rospy.loginfo_throttle(10, "The quidditch game is at {}".format(time.time()))

while not rospy.is_shutdown():
    rospy.logdebug("You're A Wizard Harry.")
    rospy.loginfo("Doris Crockford, Mr Potter. I can't believe I'm meeting you at last {}.".format(str(time.time())))
    rospy.logwarn("I solemnly swear that I am up to no good. ")
    rospy.logerr("Dumbledore will only leave from Hogwarts when there are none loyal to him!")
    rospy.logfatal("Dobby didn't mean to kill anyone. Dobby only meant to maim or seriously injure")
    rate.sleep()
    rospy.logfatal("END")

    
