#!/usr/bin/env python
# Import necessary libraries and messages
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_car import MoveMKZ

class LaneFollower(object):

    def __init__(self):
        # Initialize the objects to be used in the class
        self.bridge_object = CvBridge()
        self.move_car = MoveMKZ()
        # Get images from the rgb camera
        self.image_sub = rospy.Subscriber("/catvehicle/camera_front/image_raw_front",Image,self.camera_callback)

    def camera_callback(self,data):

        try:
            # Select bgr8 and convert image for openCV to read
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # Get image dimensions and crop the parts of the image that are not needed
        # Use different crop parameters for lane detection and obstacle/gas station detection
        height, width, channels = cv_image.shape
        descentre = 200
        des_gas = 80
        des_obstacle = 150
        des_sign = -30
        rows_to_watch = 60
        rows_watch_obstacle = 130
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        crop_gas = cv_image[(height)/2+des_gas:(height)/2+(des_gas+100)][1:width]
        crop_obstacle = cv_image[(height)/2+des_obstacle:(height)/2+(des_obstacle+rows_watch_obstacle)][1:width]
        crop_sign = cv_image[(height)/2+des_sign:(height)/2+(des_sign+100)][1:width]
        
        # Convert cropped image of lane from RGB to GRAY
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # # Convert cropped image of obstacle and stop sign from RGB to HSV
        hsv_gas = cv2.cvtColor(crop_gas, cv2.COLOR_BGR2HSV)
        hsv_obstacle = cv2.cvtColor(crop_obstacle, cv2.COLOR_BGR2HSV)
        hsv_sign = cv2.cvtColor(crop_sign, cv2.COLOR_BGR2HSV)
        # Red_hsv = [0,247,160], based on this, get the bound of gas station color
        lower_red = np.array([0,230,120])
        upper_red = np.array([0,255,280])
        # Orange_hsv = [11,255,86], based on this, get the bound of obstacle color
        lower_orange = np.array([0,230,50])
        upper_orange = np.array([20,255,110])
        # Sign_hsv = [179, 244, 23], based on this, get the bound of stop sign color
        lower_sign = np.array([150,230,0])
        upper_sign = np.array([200,255,60])

        # Threshold the HSV image to get only red and orange colors
        mask_red = cv2.inRange(hsv_gas, lower_red, upper_red)
        mask_sign = cv2.inRange(hsv_sign, lower_sign, upper_sign)
        mask_orange = cv2.inRange(hsv_obstacle, lower_orange, upper_orange)

        # Threshold the gray image to get only lane's color
        result, mask = cv2.threshold(gray,80,255,cv2.THRESH_BINARY_INV)

        # Calculate the red/orange centroid of blob in the image containing red/orange color
        m_red = cv2.moments(mask_red, False)
        try:
            cx_gas, cy_gas = m_red['m10']/m_red['m00'], m_red['m01']/m_red['m00']
        except ZeroDivisionError:
            cx_gas, cy_gas = 0, 0

        m_orange = cv2.moments(mask_orange, False)
        try:
            cx_orange, cy_orange = m_orange['m10']/m_orange['m00'], m_orange['m01']/m_orange['m00']
        except ZeroDivisionError:
            cx_orange, cy_orange = 0, 0
        
        # Determine if the dynamic obstacle is moving towards left or right
        # by finding the difference between this moment and last moment
        global cx_orange_prev
        cx_diff = cx_orange - cx_orange_prev
        cx_orange_prev = cx_orange

        # Calculate the centroid of stop sign
        m_sign = cv2.moments(mask_sign, False)
        try:
            cx_sign, cy_sign = m_sign['m10']/m_sign['m00'], m_sign['m01']/m_sign['m00']
        except ZeroDivisionError:
            cx_sign, cy_sign = 0, 0

        # Bitwise-AND mask and original image to get resulting image
        res = cv2.bitwise_and(crop_img,crop_img, mask=mask)

        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        # Track multiple blobs and choose the path to follow
        _, contours, ___ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centres = []
        cx_all = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            area = cv2.contourArea(contours[i])
            # Filter out all the reduntant contours with small areas 
            # to get the centroid of left lane and right lane
            if area > 5700:
                try:
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    cx_all.append(cx)
                    centres.append((cx, cy))
                except ZeroDivisionError:
                    pass
        
        # Draw the centroid in the resulting image
        try:
            cx_right_idx = np.argmax(cx_all)
            cx_left_idx = np.argmin(cx_all)
            cv2.circle(res, centres[cx_right_idx], 10, (0, 0, 255), -1)
            cv2.circle(res, centres[cx_left_idx], 10, (0, 0, 255), -1)
        except ValueError:
            cx_right_idx = -1
            cx_left_idx = 0
            cx_all = [0,0]
        
        ########--imshow section--#########
        # cv2.imshow("res", res_gas)
        # cv2.imshow("original",crop_gas)
        # cv2.imshow("mask",mask_red)
        # cv2.waitKey(1)
        ###################################

        # print('cx_orange= '+str(cx_orange))
        global obstacle
        global gas_station
        global obstacle_count
        # For the first obstacle, since it's moving, as long as the car detects it,
        # It's determined as an obstacle. After passing the moving obstacle,
        # only the obstacle in the right lane is considered as an obstacle
        if cx_orange > 0 and gas_station==0 and obstacle_count==0: 
            obstacle = 1
        elif cx_orange > 330 and gas_station==0:
            obstacle = 1
        
        # Keep lane following until obstacle is detected
        if obstacle == 0:
            error_x = cx_all[cx_right_idx] - width / 2 + 10;
        # If it's the first obstacle detected, it is considered as moving obstacle
        # Then after stopping for 3 seconds, wait until the obstacle is moving
        # towards the left lane and go straight to pass the obstacle
        elif obstacle_count == 0:
            print('Moving obstacle detected')
            rospy.sleep(3)
            error_x = 0
            obstacle = 0
            obstacle_count = 1
        elif obstacle_count == 1:
            if cx_orange < 330 and cx_diff < 0:
                print('GO!')
                error_x = 0
                obstacle_count = 2
                obstacle = 0
            else:
                print('Wait until the obstacle passed')
                error_x = 0
                obstacle_count = 1
                obstacle = 1
        else:
            # If obstacle detected, the error will be referenced to the obstacle
            # After taking the detour, in order to pass the obstacle clearly, 
            # the car is asked to go towards centerline if it detours to left,
            # and go towards the sideline if it detours to right
            if cx_orange != 0:
                print('Obstacle detected')
                error_x = (cx_all[cx_right_idx]-cx_orange)*6;
            elif np.abs(cx_all[cx_right_idx]-width/2) > 130 or 400<cx_all[cx_left_idx]<410:
                error_x = cx_all[cx_right_idx] - width/2 - math.copysign(130,cx_all[cx_right_idx]-width/2)
            else:
                print('Passed obstacle')
                error_x = cx_all[cx_right_idx] - width / 2 + 10
                obstacle_count = 3
                obstacle = 0

        global stop_turn
        # print('left: '+str(cx_all[cx_left_idx])+' right: '+str(cx_all[cx_right_idx]))
        # If the right lane centroid is moving to further right, prepare to make right turn
        right_turn = 1 if (cx_all[cx_right_idx]>480 or stop_turn==0) and obstacle_count==3 else 0
        twist_object = Twist();
        if right_turn == 0: 
            if obstacle == 1 and obstacle_count <= 1:
                twist_object.linear.x = 0
            # Decrease the linear velocity if error_x is too big
            elif np.abs(error_x) > 100:
                twist_object.linear.x = 1.5;
            else:
                # print('Going straight')
                twist_object.linear.x = 3.0;
            twist_object.angular.z = -error_x * 0.0018;
        else:
        # 90 degree right turn
        # When the left centroid is moving to right, starts right turning
        # until the both left and right centroid is back in the positon
            if cx_all[cx_left_idx] > 95:
                stop_turn = 0
                if cx_all[cx_right_idx]<480 and 100<cx_all[cx_left_idx]<120:
                    # print('Stop right turn')
                    stop_turn = 1
                    right_turn = 0
                else:
                    # print('Starts right turning')
                    twist_object.angular.z = -0.6
                    twist_object.linear.x = 0.8
            else:
                twist_object.angular.z = -error_x * 0.0018;
                twist_object.linear.x = 1.2

        # print('error_x=', format(error_x))

        # Stop for 3 seconds when detects the stop sign
        global stop_sign
        if cx_sign > 600 and stop_sign == 0:
            print('Stop sign detected')
            stop_sign = 1
            rospy.sleep(3)
        
        # Keep driving the car until arriving at the gas station
        # print('cx_gas=',str(cx_gas))
        if cx_gas == 0 and gas_station == 0:
            self.move_car.pub_vel(twist_object)
        else:
            # In order to park in the cnter spots, three stages are made:
            # 1. Keep turning left until the car detects the center spot
            # 2. Go towards the center spot until it's very close to it
            # 3. Keep turning right to park the car
            if gas_station == 0: gas_station = 1
            print(gas_station)
            if cx_gas != 0:
                if cx_gas < 630 and gas_station==1:
                    print('Starts turning left')
                    # print('cx_gas='+str(cx_gas))
                    error_x = cx_gas - width/2 - 300
                    twist_object.angular.z = -error_x * 0.0018;
                    twist_object.linear.x = 1.5
                    self.move_car.pub_vel(twist_object)
                elif gas_station==2:
                    if cx_gas > 220:
                        print('Moving forward')
                        # print('cx_gas='+str(cx_gas))
                        twist_object.angular.z = -0.03;
                        twist_object.linear.x = 3
                        self.move_car.pub_vel(twist_object)
                    else:
                        gas_station = 3
                elif gas_station==3:
                    print('Starts turning right')
                    # print('cx_gas='+str(cx_gas))
                    twist_object.angular.z = -0.2;
                    twist_object.linear.x = 2
                    self.move_car.pub_vel(twist_object)
                else:
                    gas_station = 2
            # Stop the car when the camera cannot see the gas station
            else: 
                print('Arrived at gas station!')
                rospy.sleep(0.1)
                self.clean_up()
                # Exit the ROS program by setting ctrl c to True
                global ctrl_c
                ctrl_c = True


    # Set car velocity to 0 and close all openCV windows
    def clean_up(self):
        self.move_car.clean_class()
        cv2.destroyAllWindows()
        
def main():
    # Initialize the line following node
    rospy.init_node('extra_credit_node', anonymous=True)
    print('Starts lane following...')
    # Intialize global variables as flags to indicate diferent states
    global stop_turn
    global gas_station
    global obstacle
    global stop_sign
    global obstacle_count
    global cx_orange_prev
    stop_sign = 0
    stop_turn = 1
    gas_station = 0
    obstacle = 0
    obstacle_count = 0
    cx_orange_prev = 0
    # Start the class
    lane_follower_object = LaneFollower()
    # Make the class stay in a loop
    rate = rospy.Rate(5)
    def shutdownhook():
        rospy.loginfo("shutdown time!")
        lane_follower_object.clean_up()
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    ctrl_c = False
    main()