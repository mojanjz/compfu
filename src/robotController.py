#!/usr/bin/env python

from __future__ import print_function
import time
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
    #This will be our state machine

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.velocity_cmd = rospy.Publisher('/R1/cmd_vel', Twist,queue_size=1)
        self.targetOffset = 450
        self.sawCrosswalk = False
        self.atCrosswalk = False
        self.offCrosswalk = True
        self.state = "driving" #CHANGE "initializing"
        self.GO_STRAIGHT = self.targetOffset
        self.TURN_LEFT = 0
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # contourToFollow = 0
        rows,cols,channels = cv_image.shape
        IMAGE_H = rows
        IMAGE_W = cols
        warped_img = cv_image[rows-200:, cols-400:cols] #CHANGE 
        crosswalkImage = cv_image[rows-200:,0:cols]
        #color masks 
        #detecting lines on the street
        lowerWhite = np.array([250, 250, 250],dtype = "uint8")
        upperWhite = np.array([255, 255, 255],dtype = "uint8")
        whiteMask = cv2.inRange(warped_img, lowerWhite, upperWhite)
        whiteMask = cv2.medianBlur(whiteMask, 5)
        whiteMask = cv2.erode(whiteMask, None, iterations=2)
        #initialization
        initWhiteMask = cv2.inRange(crosswalkImage,lowerWhite,upperWhite)
        #detecting the street
        lowerGray = np.array([50, 80, 50],dtype = "uint8")
        upperGray = np.array([190, 90, 90],dtype = "uint8")
        grayMask = cv2.inRange(warped_img, lowerGray, upperGray)
        #grass green
        lowerGreen = np.array([10,70,10],dtype = "uint8")
        upperGreen = np.array([70,210,30],dtype = "uint8")
        greenMask = cv2.inRange(warped_img, lowerGreen, upperGreen)
        #red for cross walk
        lowerRed = np.array([0, 0, 255-20],dtype = "uint8")
        upperRed = np.array([255, 20, 255],dtype = "uint8")
        redMask = cv2.inRange(crosswalkImage, lowerRed, upperRed)
        #blue for car detection
        lowerBlue = np.array([0, 0, 0],dtype = "uint8")
        upperBlue = np.array([255, 30, 20],dtype = "uint8")
        blueMask = cv2.inRange(warped_img, lowerBlue, upperBlue)
        #apply masks 
        greenOutput = cv2.bitwise_and(warped_img, warped_img, mask = greenMask)
        redOutput = cv2.bitwise_and(crosswalkImage, crosswalkImage, mask = redMask)
        grayOutput = cv2.bitwise_and(warped_img, warped_img, mask = grayMask)
        whiteOutput = cv2.bitwise_and(warped_img, warped_img, mask = whiteMask)
        blueOutput = cv2.bitwise_and(warped_img, warped_img, mask = blueMask)
        initWhiteOutput = cv2.bitwise_and(crosswalkImage, crosswalkImage, mask = initWhiteMask)
        #Find cropped images for driving
        grayWarped = cv2.cvtColor(whiteOutput,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(grayWarped, 20, 255, 0)
        img, contours, hierarchy = cv2.findContours(thresh, 1, 2)
        # bigX=0
        # cntPoints = {}
        # if(len(contours)>0):
        #     print("Num of Contours")
        #     print(len(contours))
        #     #iterate over contours
        #     for cnt in contours:
        #         #Get rid of tiny contours
        #         if(cv2.contourArea(cnt) > 2):
        #             #iterate over each point
        #             for pt in cnt:
        #                 if (pt[0][0]>bigX):
        #                     bigX = pt[0][0]
        #             print("big X:")
        #             print(bigX)
        #             cntPoints[bigX]=cnt
        #     orderedcnts = sorted(cntPoints.keys(),reverse=True)
        #     contourToFollow = cntPoints[orderedcnts[0]]
        #     # print("ordered")
        #     # print(orderedcnts)
        #     # print(cntPoints[orderedcnts[0]])
        #     # # print("cntPoints")
        #     # # print(cntPoints)


            # cv2.drawContours(whiteOutput,cntPoints[orderedcnts[0]],-1,(0,255,0),3)
            # cv2.imshow("white",whiteOutput)
            # cv2.waitKey(3)

        #check if we can see the red line indicating a cross walk
        redPercentage =float( np.count_nonzero(np.asarray(redOutput))) / float(np.count_nonzero(np.asarray(crosswalkImage)))
        initWhitePercentage = float( np.count_nonzero(np.asarray(initWhiteOutput))) / float(np.count_nonzero(np.asarray(crosswalkImage)))
        
        #Find center of mass for initialization & driving
        M = cv2.moments(img)
        middlePixel = 0
        cX = 0
        cY = 0
        if(M["m00"] == 0):
            # offset = self.targetOffset
            offset = 0 #CHECK THAT THIS WORKS 
        else:
            cX = cols - 400 + int(M["m10"]/M["m00"])
            cY = rows - 200 + int(M["m01"]/ M["m00"])
            middlePixel = cols/2
            offset = cX - middlePixel
        #Draw circles on image for troubleshooting
        cv2.circle(cv_image,(middlePixel,cY), 5, (255,0,0))
        cv2.circle(cv_image, (cX,cY), 5, (255,0,0))
        cv2.imshow("Image window", redOutput)
        cv2.waitKey(3)
        cv2.imshow("contour image",cv_image)
        cv2.waitKey(3)
        cv2.imshow("crosswalk view",crosswalkImage)

        #State machine for driving
        if(self.state == "initializing"):
            initComplete = False
            print("Initializing")
            print("White Percentage:")
            print(initWhitePercentage)
            #If white percentage is less than 20%, we haven't gone straight long enough and should keep going
            if (initWhitePercentage < 0.2):
                print("Going straight to init")
                self.pid(self.GO_STRAIGHT)
            else:
                if((abs(offset)<self.targetOffset+5) and (abs(offset)>self.targetOffset -5)):
                    print("Turning to init")
                    self.pid(self.TURN_LEFT)
                else:
                    print("Done init!")
                    initComplete = True
            if (initComplete == True):
                self.state = "driving"
            else:
                self.state = "initializing"

        elif (self.state == "driving"):
            print("Driving...")
            self.pid(offset)
            #Decide on exit state
            # if(redPercentage == 0):
            #         self.state = "driving"
            # else:
            #         self.state = "entering_crosswalk"
            
        
        elif (self.state == "entering_crosswalk"):
            print("Entering crosswalk...")
            if(redPercentage>0):
                self.state = "entering_crosswalk"
            else:
                self.state = "on_crosswalk"

        elif (self.state == "waiting_for_ped"):
            print("Waiting for pedestrian...")

        elif (self.state == "on_crosswalk"):
            print("On crosswalk...")
            if (redPercentage ==0):
                self.state = "on_crosswalk"
            else:
                self.state = "exiting_crosswalk"

        elif (self.state == "exiting_crosswalk"):
            print("Exiting crosswalk...")
            if (redPercentage > 0):
                self.state = "exiting_crosswalk"
            else:
                self.state = "driving"        
            

    def pid(self,offset):
        differenceTolerance = 55
        angularScale = 5
        xVelocity = 0.03
        zTwist = 0.0
        offsetOvershoot = self.targetOffset - offset
        if(abs(offsetOvershoot) > differenceTolerance):
            xVelocity = 0.0
            zTwist = angularScale * offsetOvershoot
        vel_msg = Twist()
        vel_msg.linear.x = xVelocity
        vel_msg.angular.z = zTwist
        self.velocity_cmd.publish(vel_msg)
    
    # def initializeAfterSpawn(self):

        


def main(args):
    rc = robot_controller()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    main(sys.argv)












