#!/usr/bin/env python
import rospy
import numpy as np
import sys
#from std_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from matplotlib import pyplot as plt

bridge = CvBridge()
i=0
def callback(data):
    global i
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")   #use bgr8 instead of passthrough otherwise the orange color tape will appear to be blue.
    #cv2.imshow('image_from_camera',cv_image)
    blur = cv2.GaussianBlur(cv_image,(5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    np.set_printoptions(threshold=np.nan)
    cv2.imwrite("yellow_tape_bad_lighting"+str(i)+".png",cv_image)
    i=i+1
    cv2.waitKey(0)
    lower_range = np.array([25, 0, 215], dtype=np.uint8)
    upper_range = np.array([38, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask) 
    talker(mask)


def listener():
    rospy.init_node('image_listener',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw",Image,callback)
    rospy.spin()
    

def talker(mask):
    pub = rospy.Publisher('masked_images',Image)
   # rospy.init_node('binary_image_publisher',anonymous=True)
    pub.publish(bridge.cv2_to_imgmsg(mask))

if __name__ == '__main__':
    listener()
