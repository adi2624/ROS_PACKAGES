#!/usr/bin/env python
import numpy as np
import rospy
import std_msgs.msg as Twist
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

bridge = CvBridge()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data)
    blur = cv2.GaussianBlur(cv_image,(5,5),0)
    #cv2.imwrite("Blur.png",blur)
    ret,thresh = cv2.threshold(blur,127,200,cv2.THRESH_BINARY)  #VERY IMPORTANT TO KEEP THRESH_BINARY. DO NOT CHANGE! Need Black background and white object to find contours.
   # cv2.imshow('Threshold IMage.png',thresh)
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
        if cx<=200:
            print("Turn Left!")
        if cx>200 and cx<280:
            print("On Track!")
        if cx>=280:
            print("Turn Right!")
        else:
            print("I don't see the line.")
        cv2.imshow('frame',cv_image)
        #talker(cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
    else:
        print("I don't see the line.")
    
        

    
def listener():
    rospy.init_node('line_following_node',anonymous=True)
    rospy.Subscriber('/masked_images',Image,callback)
    rospy.spin()
def talker(cv_image):
    print("Talking")
    pub = rospy.Publisher('line_detection',Image)
    pub.publish(bridge.cv2_to_imgmsg(cv_image))  #No images are visible in Rviz. Debug.
   


if __name__ == '__main__':
    listener()