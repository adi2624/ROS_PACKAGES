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
last_hsv_values = np.array([0,0,0])
def callback(data):
    global i
    hsv_array = test(data)
    print(hsv_array.shape)
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")   #use bgr8 instead of passthrough otherwise the orange color tape will appear to be blue.
    blur = cv2.GaussianBlur(cv_image,(5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    rows = hsv.shape[0]
    cols = hsv.shape[1]

    
    mask = cv2.inRange(hsv,np.array([0,0,226]),np.array([255,255,255]))
    
    
   

    small_image = cv2.resize(mask,(32,32))

    binarized_image = cv2.bitwise_and(small_image,1)     


    #lower_range = np.array([hsv_array[0]-2,0,hsv_array[2]-20],dtype=np.uint8)
    #upper_range = np.array([hsv_array[0]+2,255,hsv_array[2]],dtype=np.uint8)
    #mask = cv2.inRange(hsv, lower_range, upper_range)

    #cv2.imshow("mask",hsv)
   # cv2.waitKey(0)
    talker(mask)
    

def listener():
    rospy.init_node('image_listener',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw",Image,callback)
    rospy.spin()
    

def talker(mask):
    pub = rospy.Publisher('masked_images',Image)
   # rospy.init_node('binary_image_publisher',anonymous=True)
    pub.publish(bridge.cv2_to_imgmsg(mask))

def binarize_image(hsv):
    isGood = False
    hue = hsv[0]
    sat = hsv[1]
    val = hsv[2]
    if val>168:
        return True
    if hue>49.5:
        return False
    if hue>33.5:
        if sat>71.5:
            return True
        return False
    if hue<=10.5:
        return False
    if sat<=40:
        return False
    if hue<=14.5:
        return False
    return True
def test(data):
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

    print("CX:"+str(cx)+" CY:" + str(cy))
    print("Corresponding HSV values are: "+str(hsv[cy][cx])) # x and y values are reversed in HSV MATRIX.
    cv2.circle(img,(cx,cy), 10, (0,0,255), -1)

    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img,[box],0,(0,0,255),2)

    rows,cols = th3.shape[:2]
    [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    print(str((cols-1))+","+str(righty))
    print(str(0)+" "+str(lefty))
    cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
    #cv2.imshow("th3",img)
    #cv2.imshow("th32",th3)
    #cv2.imshow("hsv",hsv)
    #cv2.waitKey(0)
    if(hsv[cy][cx][0]>34 or hsv[cy][cx][0]<28 or ((abs(hsv[cy][cx][0]-last_hsv_values[0]))>40) or hsv[cy][cx][2]<250  ) :          #preferably here the HSV values for the desired color would go to filter out unwanted data.
       last_hsv_values = hsv[cy][cx]
       return np.array([0,0,0])

    else:
        last_hsv_values = hsv[cy][cx]
        return hsv[cy][cx]
    

    

if __name__ == '__main__':
    listener()
