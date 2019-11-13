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
        self.prevSlope = 0

    def callback(self,data):
        infinityThresh = 1.5
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

    
        cv_image = cv_image[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        warped_img = cv2.warpPerspective(cv_image, M, (IMAGE_W, IMAGE_H)) # Image warping

        #color masks 
        #detecting lines on the street
        lowerWhite = np.array([250, 250, 250],dtype = "uint8")
        upperWhite = np.array([255, 255, 255],dtype = "uint8")
        whiteMask = cv2.inRange(warped_img, lowerWhite, upperWhite)
        # whiteMask = cv2.GaussianBlur(whiteMask, (5,5), 0)
        whiteMask = cv2.medianBlur(whiteMask, 19)
        whiteMask = cv2.erode(whiteMask, None, iterations=2)
	# whiteMask = cv2.dilate(whiteMask, None, iterations=2)
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
        imgray = cv2.cvtColor(whiteOutput, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        _,contours,_  = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(whiteOutput, contours, -1, (255,0,0), 3)
        blueOutput = cv2.bitwise_and(warped_img, warped_img, mask = blueMask)
        # print(contours)
        edges = cv2.Canny(whiteOutput, 50, 150)
        lines = cv2.HoughLinesP(edges,1,np.pi/180, 20, 100)
        m = [] #list for slopes of the detected lines 

        STRAIGHT = 4000
        if (lines is None):
            self.pid(STRAIGHT) #go straight
        else:
            for line in lines:
                for points in line:
                    m.append(float (points[1] - points[3]) / (points[0] - points[2])) #  m = y/x
            m = [x for x in m if str(x) != 'nan'] #gets rid of nan values
            # print(m)
            # slopesWeighted = self.mergeSlopes(m)
            # print("weighted slopes:")
            # print(slopesWeighted)
            longestLength = 0
            longestLine = [0,0,0,0]
            for i in range(len(lines)):
                for x1,y1,x2,y2 in lines[i]:
                    length = ((x2-x1)**2 + (y2-y1)**2)**(0.5)
                    if(length > longestLength):
                        longestLength = length
                        longestLine = [x1,y1,x2,y2]
                    # cv2.line(whiteOutput,(x1,y1), (x2,y2), (0,255,0), 2)
            cv2.line(whiteOutput,(longestLine[0],longestLine[1]),(longestLine[2],longestLine[3]), (0,0,255), 2)
            longestLineSlope = float((longestLine[1] - longestLine[3])/(longestLine[0] - longestLine[2]))
            print("longest line slope:")
            print(longestLineSlope)
            # averageSlope = longestLineSlope
            sumOfSlopes = 0
            numOfSlopes = 0
            numberOfInfinity = 0
            for slope in m:
                if slope == float("-inf") or slope == float("inf"):
                    numberOfInfinity = numberOfInfinity + 1
                else:
                    sumOfSlopes = sumOfSlopes + slope
                    numOfSlopes = numOfSlopes + 1
            # print("slopes")
            print (m)
            print ("sum of slopes")
            print (sumOfSlopes)
            print("num of slopes")
            print(numOfSlopes)
            print("num of inf slopes")
            print(numberOfInfinity)
            if(numberOfInfinity > infinityThresh*numOfSlopes): #we should go straight
                averageSlope = float("inf")
            else: 
                averageSlope = sumOfSlopes / numOfSlopes #we should turn
            print("average slope")
            print(averageSlope)

            #drawing the hough lines on the white mask
            cv2.line(whiteOutput,(int(cols/2),int(rows/2)), (int(20 / averageSlope + cols/2), int(rows/2 + 20)),(255,0,0),2)

            cv2.imshow("whiteMask",whiteOutput)
            cv2.waitKey(3)
            cv2.imshow("Image window", warped_img)
            cv2.waitKey(3)

            self.pid(averageSlope)

    def pid(self,slope):
        slopeThresh = 5000
        angularScale = 5
        xVelocity = 0.03
        zTwist = 0.0
        if(abs(slope) < slopeThresh and abs(self.prevSlope) > slopeThresh):
            xVelocity = 0.0 
            zTwist = float(angularScale  / slope)
            self.prevSlope = slope 
        else:
            self.prevSlope = float("inf")
        vel_msg = Twist()
        vel_msg.linear.x = xVelocity
        vel_msg.angular.z = zTwist
        # print (vel_msg)
        self.velocity_cmd.publish(vel_msg)
    def mergeSlopes (self,slopes):
        slopeTolerance = 1
        weightedSlopes = {"slope": [], "number" : []}
        weightedSlopes["slope"].append(float("inf"))
        weightedSlopes["number"].append(0)
        weightedSlopesCopy = weightedSlopes
        # weightedSlopes["slope"].append(slopes.pop(0))
        # weightedSlopes["number"].append(1)
        for i, slope in enumerate(slopes):
            print("slope in merge slope")
            print(slope)
            print("index")
            print(i)
            for index , diffSlope in enumerate(weightedSlopes["slope"]):
                print("diff slope:")
                print (diffSlope)
                print("difference in slope")
                print(abs(float(slope) - diffSlope))
                print("index of the number")
                print(index)
                if (slope == float("inf") or slope == float("-inf")):
                    print("infinity slope")
                    weightedSlopes["number"][0] += 1
                elif (abs(float(slope) - diffSlope) < slopeTolerance):
                    weightedSlopes["number"][index] += 1
                else:
                    weightedSlopes["slope"].append(slope)
                    weightedSlopes["number"].append(1)
            print(weightedSlopes)
        return weightedSlopes
                    





        


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












