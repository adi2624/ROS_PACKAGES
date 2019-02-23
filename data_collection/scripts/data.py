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
bridge = CvBridge()
last_hsv_values = np.array([0,0,0])
image_path = '/home/nvidia/data/images/'
vel_path = '/home/nvidia/data/cmd_vel/'

def image_callback(data):
    global last_file_number,next_file_number,latest_cmd_vel,image_path,vel_path,cv_bridge
    
    # First off, if the latest cmd_vel was not moving, don't save any data (we don't want to train the robot to not move!)
    if latest_cmd_vel.linear.x==0 and latest_cmd_vel.angular.z==0:
        return
    
    hsv_array = hsv_filter(data)
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")   #use bgr8 instead of passthrough otherwise the orange color tape will appear to be blue.
    blur = cv2.GaussianBlur(cv_image,(5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_range = np.array([hsv_array[0]-2,0,hsv_array[2]-20],dtype=np.uint8)
    upper_range = np.array([hsv_array[0]+2,255,hsv_array[2]],dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)       # converted black and white image
    small_image = cv2.resize(mask,(32,32))    # resize to a 32x32 image
    if last_file_number==-1:
        files = os.listdir("/home/nvidia/data/cmd_vel")
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
    with open(os.path.join(vel_path, next_vel_file),"w") as f:
        f.write('{}\t{}'.format(latest_cmd_vel.linear.x,latest_cmd_vel.angular.z))
    f.close()
    # Increment file number
    last_file_number = next_file_number

def cmd_vel_callback(data):
    global latest_cmd_vel
    latest_cmd_vel = data;


def hsv_filter(data):

    global last_hsv_values
    original_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
    img = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
  #  img = cv2.imread('/home/aditya/catkin_ws/src/image_masking/scripts/yellow_tape_good_lighting74.png',0)
    hsv = cv2.cvtColor(original_image,cv2.COLOR_BGR2HSV)
    #Otsu's Binarization on Grayscale to detect X,Y positions of line.

    blur = cv2.GaussianBlur(img,(5,5),0)

    ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    im2,contours,hierarchy = cv2.findContours(th3, 1, 2)

    if len(contours)>0:
        c = max(contours,key=cv2.contourArea)
        cnt = c
    
    M = cv2.moments(cnt)

    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    #print("CX:"+str(cx)+" CY:" + str(cy))
    #print("Corresponding HSV values are: "+str(hsv[cy][cx])) # x and y values are reversed in HSV MATRIX.
    cv2.circle(img,(cx,cy), 10, (0,0,255), -1)

    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img,[box],0,(0,0,255),2)

    rows,cols = th3.shape[:2]
    [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    #print(str((cols-1))+","+str(righty))
    #print(str(0)+" "+str(lefty))
    cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
    #cv2.imshow("th3",img)
    #cv2.imshow("th32",th3)
    #cv2.imshow("hsv",hsv)
    #cv2.waitKey(0)
    if(hsv[cy][cx][0]>34 or hsv[cy][cx][0]<28 or ((abs(int(hsv[cy][cx][0])-int(last_hsv_values[0])))>40) or hsv[cy][cx][2]<250  ) :          #preferably here the HSV values for the desired color would go to filter out unwanted data.
       last_hsv_values = hsv[cy][cx]
       return np.array([0,0,0])
    else:
        last_hsv_values = hsv[cy][cx]
        return hsv[cy][cx]


rospy.init_node('data_collection_node',anonymous=True)
rospy.Subscriber('/camera/color/image_raw',Image,image_callback)
rospy.Subscriber('/cmd_vel',Twist,cmd_vel_callback)
rospy.spin()
