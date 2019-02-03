#!/usr/bin/env python
import rospy
import numpy as np
import sys
#from std_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

bridge = CvBridge()

def callback(data):
    
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
    height, width = cv_image.shape[:2]
    cropped_image = cv_image  #use bgr8 instead of passthrough otherwise the orange color tape will appear to be blue.
    cv2.imwrite("line.jpg", cropped_image)
    #cv2.imshow('image_from_camera',cv_image)
    apply_box_detection(cropped_image)
    '''
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_range = np.array([3, 100, 100], dtype=np.uint8)
    upper_range = np.array([23, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)
    cv2.imshow('mask',mask)
    #cv2.waitKey(0)
    talker(mask)
    '''

def listener():
    rospy.init_node('image_listener',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw",Image,callback)
    rospy.spin()
    

def talker(mask):
    pub = rospy.Publisher('masked_images',Image)
   # rospy.init_node('binary_image_publisher',anonymous=True)
    pub.publish(bridge.cv2_to_imgmsg(mask))

def apply_box_detection(cv_image):
    gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    cv2.imshow('Gray',gray)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    cv2.imshow("blur",blur)
    ret,thresh = cv2.threshold(blur,127,200,cv2.THRESH_BINARY_INV)
    cv2.imshow('Threshold IMage',thresh)
    _,contours,hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if(len(contours))>0:
        c = max(contours,key=cv2.contourArea)
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.line(cv_image,(cx,0),(cx,480),(255,0,0),1)
        cv2.line(cv_image,(0,cy),(640,cy),(255,0,0),1)
        print("cx:",cx)
        cv2.drawContours(cv_image, contours, -1, (0,255,0), 1)
        if cx>=120:
            print("Turn Left!")
        if cx<120 and cx>50:
            print("On Track!")
        if cx<=50:
            print("Turn Right!")
        else:
            print("I don't see the line.")
        cv2.imshow('frame',cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


if __name__ == '__main__':
    listener()