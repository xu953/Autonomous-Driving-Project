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
        des_obstacle = 130
        rows_to_watch = 60
        rows_watch_obstacle = 130
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        crop_obstacle = cv_image[(height)/2+des_obstacle:(height)/2+(des_obstacle+rows_watch_obstacle)][1:width]
        
        # Convert cropped image of lane from RGB to GRAY
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # Convert cropped image of obstacle from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_obstacle = cv2.cvtColor(crop_obstacle, cv2.COLOR_BGR2HSV)
        # Red_hsv = [0,247,160], based on this, get the bound of red color
        lower_red = np.array([0,230,120])
        upper_red = np.array([0,255,280])
        # Orange_hsv = [11,255,86], based on this, get the bound of orange color
        lower_orange = np.array([0,230,50])
        upper_orange = np.array([20,255,110])
        # Threshold the HSV image to get only red and orange colors
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_orange = cv2.inRange(hsv_obstacle, lower_orange, upper_orange)

        # Threshold the gray image to get only lane's color
        result, mask = cv2.threshold(gray,80,255,cv2.THRESH_BINARY_INV)

        # Calculate the red/orange centroid of blob in the image containing red/orange color
        m_red = cv2.moments(mask_red, False)
        try:
            cx_red, cy_red = m_red['m10']/m_red['m00'], m_red['m01']/m_red['m00']
        except ZeroDivisionError:
            cx_red, cy_red = width/2, height/2

        m_orange = cv2.moments(mask_orange, False)
        try:
            cx_orange, cy_orange = m_orange['m10']/m_orange['m00'], m_orange['m01']/m_orange['m00']
        except ZeroDivisionError:
            cx_orange, cy_orange = 0, 0
        # print('cx_orange= '+str(cx_orange))

        # Calculate centroid of the blob of lane using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask=mask)

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
        cx_right_idx = np.argmax(cx_all)
        cx_left_idx = np.argmin(cx_all)
        cv2.circle(res, centres[cx_right_idx], 10, (0, 0, 255), -1)
        cv2.circle(res, centres[cx_left_idx], 10, (0, 0, 255), -1)

        #######-imshow section-#######
        # cv2.imshow("res", res)
        # cv2.imshow("gray",gray)
        # cv2.imshow("mask",mask)
        # cv2.waitKey(1)
        ##############################

        global obstacle
        global gas_station
        # Only the obstacle in the right lane is considered as obstacle
        if cx_orange > 180 and gas_station==0: obstacle = 1

        # When no obstacle is detected, keep following the lane
        # Otherwise, the error will be referenced to the obstacle
        # After taking the detour, in order to pass the obstacle clearly, 
        # the car is asked to go towards centerline if it detours to left,
        # and go towards the sideline if it detours to right
        if obstacle == 0:
            error_x = cx_all[cx_right_idx] - width / 2 + 10;
        else:
            if cx_orange != 0:
                print('Obstacle detected')
                error_x = (cx_all[cx_right_idx]-cx_orange)*6;
            elif np.abs(cx_all[cx_right_idx]-width/2) > 130 or 400<cx_all[cx_left_idx]<410:
                error_x = cx_all[cx_right_idx] - width/2 - math.copysign(130,cx_all[cx_right_idx]-width/2)
            else:
                print('Passed obstacle')
                error_x = cx_all[cx_right_idx] - width / 2 + 10
                obstacle = 0
        
        global stop_turn
        # If the right lane centroid is moving to further right, prepare to make right turn
        right_turn = 1 if (cx_all[cx_right_idx]>480 or stop_turn==0) and obstacle==0 else 0
        twist_object = Twist();
        if right_turn == 0: 
            # Decrease the linear velocity if error_x is too big
            if np.abs(error_x) > 100:
                twist_object.linear.x = 1.5;
            else:
                # print('Going straight')
                twist_object.linear.x = 3;
            twist_object.angular.z = -error_x * 0.0018;
        else:
        # 90 degree right turn
        # When the left centroid is moving to right, starts right turning
        # until the both left and right centroid is back in the positon
            if cx_all[cx_left_idx] > 95:
                stop_turn = 0
                if cx_all[cx_right_idx]<480 and 100<cx_all[cx_left_idx]<120:
                    # print('Stop turn')
                    stop_turn = 1
                else:
                    # print('Starts right turning')
                    twist_object.angular.z = -0.6
                    twist_object.linear.x = 0.8
            else:
                twist_object.angular.z = -error_x * 0.0018;
                twist_object.linear.x = 1.2

        # print('error_x=', format(error_x))
       
        # Make it start moving until the car reaches the gas station
        if cx_red == width/2 and gas_station==0:
            self.move_car.pub_vel(twist_object)
        else:
            gas_station = 1
            # After gas station is detected, keep going straight until
            # the camera cannot see the gas station
            if cx_red != width/2:
                self.move_car.pub_vel(twist_object)
            else: 
                print('Arrived at gas station!')
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
    rospy.init_node('basic_task_node', anonymous=True)
    print('Starts lane following...')
    # Intialize global variables as flags to indicate diferent states
    global stop_turn
    global gas_station
    global obstacle
    stop_turn = 1
    gas_station = 0
    obstacle = 0
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