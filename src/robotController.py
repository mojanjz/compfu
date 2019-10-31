#!/usr/bin/env python

from __future__ import print_function

import roslib
import numpy as np
import math
#roslib.load_manifest('my_package')
import sys
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 


class robot_controller:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.velocity_cmd = rospy.Publisher('/R1/cmd_vel', Twist,queue_size=1)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        rows,cols,channels = cv_image.shape

        IMAGE_H = rows
        IMAGE_W = cols

        #birdeye view of the image
        src = np.float32([[0, IMAGE_H], [1207, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        img = cv_image
        img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping


        #color masks 
        #detecting lines on the street
        lowerWhite = np.array([0, 200, 0],dtype = "uint8")
        upperWhite = np.array([255, 255, 255],dtype = "uint8")
        whiteMask = cv2.inRange(warped_img, lowerWhite, upperWhite)
        #detecting the street
        lowerGray = np.array([0, 0, 50],dtype = "uint8")
        upperGray = np.array([190, 90, 90],dtype = "uint8")
        grayMask = cv2.inRange(warped_img, lowerGray, upperGray)
        #grass green
        lowerGreen = np.array([10,70,10],dtype = "uint8")
        upperGreen = np.array([70,210,30],dtype = "uint8")
        greenMask = cv2.inRange(warped_img, lowerGreen, upperGreen)
        #red for cross walk
        lowerRed = np.array([0, 0, 255-20],dtype = "uint8")
        upperRed = np.array([255, 20, 255],dtype = "uint8")
        redMask = cv2.inRange(warped_img, lowerRed, upperRed)
        #blue for car detection
        lowerBlue = np.array([0, 0, 0],dtype = "uint8")
        upperBlue = np.array([255, 30, 20],dtype = "uint8")
        blueMask = cv2.inRange(warped_img, lowerBlue, upperBlue)
        #apply masks for lane detection
        greenOutput = cv2.bitwise_and(warped_img, warped_img, mask = greenMask)
        redOutput = cv2.bitwise_and(warped_img, warped_img, mask = redMask)
        greyOutput = cv2.bitwise_and(warped_img, warped_img, mask = grayMask)
        whiteOutput = cv2.bitwise_and(warped_img, warped_img, mask = whiteMask)
        blueOutput = cv2.bitwise_and(warped_img, warped_img, mask = blueMask)
        
        rowW,colW,rgbW = np.nonzero(whiteOutput)
        mostFrequentColumnW = np.argmax(np.bincount(colW))

        rowGY, colGY, rbgGY = np.nonzero(greyOutput)
        mostFrequentColumnGY = np.argmax(np.bincount(colGY))

        print("most frequent col:")
        print (mostFrequentColumnW)
        print("most frequent col grey:")
        print (mostFrequentColumnGY)

        targetLocation = 0
        lineOffset = 120
        if(mostFrequentColumnGY > mostFrequentColumnW):
            targetLocation = mostFrequentColumnW + lineOffset
        else :
            targetLocation = mostFrequentColumnW - lineOffset
        # print("non zero columns")
        # print(col)
        # print ("hey")
        # print(whiteOutput)
        # print("non zero")
        print(np.nonzero(whiteOutput))


        cv2.circle(warped_img,(targetLocation,20),10,255)
        cv2.circle(warped_img,(mostFrequentColumnGY,20),10,0)
        cv2.circle(warped_img,(mostFrequentColumnW,20),10,120)
        height,width,channels = warped_img.shape
        middlePixel = width / 2
        cv2.circle(warped_img,(middlePixel,20),10,200)
        print("middle pixel")
        print(middlePixel)

        if(middlePixel > targetLocation):
            print("turning left")
        else:
            print("turning right")

        cv2.imshow("whiteMask",whiteOutput)
        cv2.waitKey(3)
        cv2.imshow("Image window", warped_img)
        cv2.waitKey(3)

        # self.pid(targetLocation,middlePixel)

    def pid(self,centreOfMass,middlePixel):
        twistThreshold = 10
        zTwist = 0.1
        xVelocity = 0.03
        xDifference = centreOfMass - middlePixel
        if(xDifference > twistThreshold ):
            xVelocity = 0.0 
        else:
            zTwist = 0.0
        #xDifference>0 -> line on right
        vel_msg = Twist()
        vel_msg.linear.x = xVelocity
        vel_msg.angular.z = -zTwist * xDifference
        print (vel_msg)
        self.velocity_cmd.publish(vel_msg)
        


def main(args):
    rospy.init_node('robot_controller', anonymous=True)
    rc = robot_controller()
    # try:
    rospy.spin()
    # except KeyboardInterrupt:
        # print("Shutting down")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)












