#!/usr/bin/env python
# 
# Author: Jonathan Sprinkle
# Copyright (c) 2015-2016 Arizona Board of Regents
# All rights reserved.
# 
# Permission is hereby granted, without written agreement and without 
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the 
# above copyright notice and the following two paragraphs appear in 
# all copies of this software.
# 
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
# IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
# SUCH DAMAGE.
# 
# THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
# IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
# TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# This node converts cmd_vel inputs to the vehicle to the ROS topics that
# are exposed in Gazebo for moving the vehicle in simulation. Notably, the
# inputs to Gazebo are to joints on the wheel, so there is a multiplier of
# 2.8101 that is applied to the joint's velocity whenever we try to move
# so that the output in Gazebo will match the desired input velocity.

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
import sys, getopt, math

class cmdvel2gazebo:

    def __init__(self,ns):
        self.ns = ns
        rospy.init_node('cmdvel2gazebo', anonymous=True)

        # the format(ns) looks for the namespace in the ros parameter server, I guess
        
        rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.pub_steerL = rospy.Publisher('/catvehicle/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/catvehicle/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('/catvehicle/joint1_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/catvehicle/joint2_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)
        # simulator value matches the 'real' car
        self.L = 2.62

        # car Tread
        # this value is from the car's manual
        # self.T=1.55

        # car Tread
        # this value is from the simulator
        self.T = 1.301

        # how many seconds delay for the dead man's switch
        # set timeout from launch file or rosparam
        self.timeout=rospy.Duration.from_sec(0.2);
        self.lastMsg=rospy.Time.now()

        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        self.maxsteerInside=0.6;
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside);
        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.T/2.0)
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)
        # the ideal max steering angle we can command is now set
        rospy.logwarn("######### MAXIMUM ideal steering angle set to =="+str(self.maxsteer))
        

    def callback(self,data):
        # 2.8101 is the gain factor in order to account for mechanical reduction of the tyres
        self.x = 2.8101*data.linear.x
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            #rospy.loginfo(rospy.get_caller_id() + " timed out waiting for new input in /cmd_vel, setting velocity to 0.")
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x;
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            return
        #rospy.loginfo("X = "+ str(self.x)+ ", Z = "+ str(self.z))
        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            T=self.T
            L=self.L
            # self.v is the linear *velocity*
            r = L/math.fabs(math.tan(self.z))

            rL = r-(math.copysign(1,self.z)*(T/2.0));
            rR = r+(math.copysign(1,self.z)*(T/2.0))
            msgRearR = Float64()
            # the right tire will go a little faster when we turn left (positive angle)
            # amount is proportional to the radius of the outside/ideal
            msgRearR.data = self.x*rR/r;
            msgRearL = Float64()
            # the left tire will go a little slower when we turn left (positive angle)
            # amount is proportional to the radius of the inside/ideal
            msgRearL.data = self.x*rL/r;

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            # the left tire's angle is solved directly from geometry
            msgSteerL.data = math.atan2(L,rL)*math.copysign(1,self.z)
            self.pub_steerL.publish(msgSteerL)
    
            # the right tire's angle is solved directly from geometry
            msgSteerR.data = math.atan2(L,rR)*math.copysign(1,self.z)
            self.pub_steerR.publish(msgSteerR)
        else:
            # if we aren't turning, everything is easy!
            msgRear = Float64()
            msgRear.data = self.x;
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)
def usage():
    print('cmdvel2gazebo -n catvehicle')


def main(argv):
    # we eventually get the ns (namespace) from the ROS parameter server for this node
    ns=''
    node = cmdvel2gazebo(ns)
    rate = rospy.Rate(10) # run at 10Hz
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
    try:
        listener('catvehicle')
    except rospy.ROSInterruptException:
        pass


