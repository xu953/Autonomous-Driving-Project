#!/usr/bin/env python
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')

    state_msg = ModelState()

    state_msg.model_name = 'catvehicle'
    state_msg.pose.position.x = 0  #151.5 #90 #180 #103 #112
    state_msg.pose.position.y = -1.8   #90 #-1.8 #98.5 #0.8 #9.5
    state_msg.pose.position.z = 0.0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0 #1 #1
    state_msg.pose.orientation.w = 0 #1 #2.5

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass