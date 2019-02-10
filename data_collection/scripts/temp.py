#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import oss
import sys 
from std_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

latest_cmd_vel = 0
last_file_number = -1

def cmd_vel_callback(data):
	latest_cmd_vel = data

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
	cmd_vel = latest_cmd_vel
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_range = np.array([3, 100, 100], dtype=np.uint8)
    upper_range = np.array([23, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)

    if global last_file_number==-1:
        files = os.listdir("/home/nivida/data/vel")
        files.sort()
        last_file = files[-1]
        last_file_number = int(last_file.split('.')[0])
    next_file_number = last_file_number+1
    next_data_file = str(next_file_number).zfill(6)+".txt"
    next_image_file = str(next_file_number).zfill(6)+".jpg"
    # SAVE FILES HERE
    last_file_number = next_file_number
    cv2.imwrite(next_image_file,mask)
    file_object = open(next_data_file,"w")
    file_object.write(latest_cmd_vel)
	# Grab Imag
def listener():
    rospy.init_node('grabber',anonymous=True)
	rospy.Subscribe("/cmd_vel", Twist, cmd_vel_callback)
	rospy.Subscribe("/camera/color/raw_image", Image, image_callback)
