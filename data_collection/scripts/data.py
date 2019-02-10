#!/usr/bin/env python
import os
import sys
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np

latest_cmd_vel = 0
last_file_number = -1
next_file_number = 0
cv_bridge = CvBridge()

def listener():
    rospy.init_node('data_collection_node',anonymous=True)
    rospy.Subscriber('/camera/color/image_raw',Image,image_callback)
    rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
    rospy.spin()

def image_callback(data):
    global last_file_number,next_file_number
    image = cv_bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_range = np.array([3, 100, 100], dtype=np.uint8)
    upper_range = np.array([23, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)       # converted black and white image
    small_image = cv2.resize(mask,(32,32))    # resize to a 32x32 image
    if last_file_number==-1:
        files = os.listdir("/home/aditya/data/cmd_vel")
        files.sort()
        last_file = files[-1]
        last_file_number = int(last_file.split('.')[0])
    next_file_number = last_file_number+1
    next_image_file = str(next_file_number).zfill(6)+".jpg"
    last_file_number = next_file_number
    path = '/home/aditya/data/images'
    cv2.imwrite(os.path.join(path ,next_image_file),small_image)
    #cv2.imshow("data.png",small_image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

def cmd_vel_save(file_number,path,data):
    file_object = open(os.path.join(path,file_number),"w")
    file_object.write(str(data))
    file_object.close()

def cmd_vel_callback(data):
    cmd_vel_save(str(next_file_number).zfill(6)+".txt",'/home/aditya/data/cmd_vel',data)






listener()