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
image_path = '/home/aditya/data/'
vel_path = '/home/aditya/data/vel/'

def image_callback(data):
    global last_file_number,next_file_number,latest_cmd_vel,image_path,vel_path,cv_bridge
    
    # First off, if the latest cmd_vel was not moving, don't save any data (we don't want to train the robot to not move!)
    if latest_cmd_vel.linear.x==0 and latest_cmd_vel.angular.z==0:
        return
    
    image = cv_bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    lower_range = np.array([3, 100, 100], dtype=np.uint8)
    upper_range = np.array([23, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)       # converted black and white image
    small_image = cv2.resize(mask,(32,32))    # resize to a 32x32 image
    if last_file_number==-1:
        files = os.listdir("/home/aditya/data/cmd_vel")
        files.sort()
        if not files: # If no files in the directory
            last_file_number = -1
        else:
            last_file_number = int(files[-1].split('.')[0])
    next_file_number = last_file_number+1
    
    # Determine File Names
    next_image_file = image_path+str(next_file_number).zfill(6)+".jpg"
    next_vel_file = vel_path+str(next_file_number).zfill(6)+".txt"
    
    # Save Files
    cv2.imwrite(next_image_file,small_image)
    with open(next_vel_file) as f:
        f.write('{}\t{}'.format(latest_cmd_vel.linear.x,latest_cmd_vel.angular.z))
    
    # Increment file number
    last_file_number = next_file_number

def cmd_vel_callback(data):
    global latest_cmd_vel
    latest_cmd_vel = data;

rospy.init_node('data_collection_node',anonymous=True)
rospy.Subscriber('/camera/color/image_raw',Image,image_callback)
rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
rospy.spin()
