#!/usr/bin/env python
# Import necessary libraries and messages
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class MoveMKZ(object):

    def __init__(self):
        # Initialize the publisher for velocity and joint positions
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', \
            Twist, queue_size=1)


    # Drive the robot by publishing twist_object
    def pub_vel(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
    

    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.linear.x = 0.0
        twist_object.angular.z = 0.0
        self.pub_vel(twist_object)
        self.shutdown_detected = True