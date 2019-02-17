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
    hsv_array = test(data)
    print(hsv_array.shape)
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")   #use bgr8 instead of passthrough otherwise the orange color tape will appear to be blue.
    blur = cv2.GaussianBlur(cv_image,(5,5),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_range = np.array([hsv_array[0]-5,0,hsv_array[2]-20],dtype=np.uint8)
    upper_range = np.array([hsv_array[0]+5,255,hsv_array[2]],dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_range, upper_range)
    #cv2.imshow("mask",mask)
    #cv2.waitKey(0)
    talker(mask)
    

def listener():
    rospy.init_node('image_listener',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw",Image,callback)
    rospy.spin()
    

def talker(mask):
    pub = rospy.Publisher('masked_images',Image)
   # rospy.init_node('binary_image_publisher',anonymous=True)
    pub.publish(bridge.cv2_to_imgmsg(mask))

def test(data):

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
    if((hsv[cy][cx][0]>38 or hsv[cy][cx][0]<25) or (hsv[cy][cx][2]<215 or hsv[cy][cx][2]>255) ):          #preferably here the HSV values for the desired color would go to filter out unwanted data.
        return np.array([0,0,0])
    else:
        return hsv[cy][cx]

    

if __name__ == '__main__':
    listener()
