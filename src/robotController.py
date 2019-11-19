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

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        self.velocity_cmd = rospy.Publisher('/R1/cmd_vel', Twist,queue_size=1)
        self.targetOffset = 450
        self.sawCrosswalk = False
        self.atCrosswalk = False
        self.offCrosswalk = True

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        rows,cols,channels = cv_image.shape

        IMAGE_H = rows
        IMAGE_W = cols

        # #birdeye view of the image
        # src = np.float32([[0, IMAGE_H], [1207, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        # dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        # M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        # Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

    
        # cv_image = cv_image[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        # warped_img = cv2.warpPerspective(cv_image, M, (IMAGE_W, IMAGE_H)) # Image warping
        warped_img = cv_image[rows-200:, cols-400:cols] #CHANGE 
        crosswalkImage = cv_image[rows-200:,0:cols]
        #color masks 
        #detecting lines on the street
        lowerWhite = np.array([250, 250, 250],dtype = "uint8")
        upperWhite = np.array([255, 255, 255],dtype = "uint8")
        whiteMask = cv2.inRange(warped_img, lowerWhite, upperWhite)
        # whiteMask = cv2.blur(whiteMask, (5,5))
        whiteMask = cv2.medianBlur(whiteMask, 5)
        whiteMask = cv2.erode(whiteMask, None, iterations=2)
	# whiteMask = cv2.dilate(whiteMask, None, iterations=2)
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
        #apply masks for lane detection
        greenOutput = cv2.bitwise_and(warped_img, warped_img, mask = greenMask)
        redOutput = cv2.bitwise_and(crosswalkImage, crosswalkImage, mask = redMask)
        grayOutput = cv2.bitwise_and(warped_img, warped_img, mask = grayMask)
        whiteOutput = cv2.bitwise_and(warped_img, warped_img, mask = whiteMask)
        blueOutput = cv2.bitwise_and(warped_img, warped_img, mask = blueMask)

        offset = 0
        redPercentage =float( np.count_nonzero(np.asarray(redOutput))) / float(np.count_nonzero(np.asarray(crosswalkImage)))
        if(redPercentage > 0.0 and self.offCrosswalk == True):
            print("saw a crosswalk")
            self.sawCrosswalk = True
            self.offCrosswalk = False
        elif(self.sawCrosswalk == True and redPercentage == 0.0):
            print("on crosswalk")
            self.atCrosswalk = True
            self.offCrosswalk = False
        elif(self.atCrosswalk == True and redPercentage > 0.0):
            print("going off cross walk")
        elif(self.atCrosswalk == True and redPercentage == 0.0):
            print("passed the crosswalk")
            self.offCrosswalk == True
        if(self.offCrosswalk == True):
            print("off the cross walk")
            self.sawCrosswalk = False
            self.atCrosswalk = False

        
            
            
        
        grayWarped = cv2.cvtColor(whiteOutput,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(grayWarped, 20, 255, 0)
        img, contours, hierarchy = cv2.findContours(thresh, 1, 2)
        #find center of mass
        M = cv2.moments(img)
        middlePixel = 0
        cX = 0
        cY = 0
        if(M["m00"] == 0):
            offset = self.targetOffset
        # elif(self.atCrosswalk == True):
        #     offset = self.targetOffset
        else:
            cX = cols - 400 + int(M["m10"]/M["m00"])
            cY = rows - 200 + int(M["m01"]/ M["m00"])
            middlePixel = cols/2
            offset = cX - middlePixel
            # print("current offset:")
            # print(offset)
            # print("target offset: ")
            # print(self.targetOffset)
        cv2.circle(cv_image,(middlePixel,cY), 5, (255,0,0))
        cv2.circle(cv_image, (cX,cY), 5, (255,0,0))
        # cv2.imshow("whiteMask",whiteOutput)
        # cv2.waitKey(3)

        cv2.imshow("Image window", redOutput)
        cv2.waitKey(3)
        cv2.imshow("contour image",cv_image)
        cv2.waitKey(3)

        self.pid(offset)

    def pid(self,offset):
        # slopeThresh = 5000
        differenceTolerance = 55
        angularScale = 5
        xVelocity = 0.03
        zTwist = 0.0
        offsetOvershoot = self.targetOffset - offset
        # print("offset overshoot")
        # print(offsetOvershoot )
        if(abs(offsetOvershoot) > differenceTolerance):
            # print("turning")
            # if(offsetOvershoot > 0):
            #     print ("right")
            # else:
            #     print("left")
            xVelocity = 0.0
            zTwist = angularScale * offsetOvershoot
        vel_msg = Twist()
        vel_msg.linear.x = xVelocity
        vel_msg.angular.z = zTwist
        # print (vel_msg)
        self.velocity_cmd.publish(vel_msg)

def initializeAfterSpawn():
    straightTime = 0.8
    turnTime = 0.05
    #initialize velocity publishing
    velocity_cmd = rospy.Publisher('/R1/cmd_vel', Twist,queue_size=1)
    # print("in the initialization spawn")
    #start tracking time
    startTime = time.time()
    #CHANGE : For now assume that we are spawned in the time trials location
    while((time.time()-startTime)<straightTime):
        # print("yeeting forward")
        #go straight
        vel_msg = Twist()
        vel_msg.linear.x = 1
        vel_msg.angular.z = 0
        velocity_cmd.publish(vel_msg)
    startTime = time.time()
    while((time.time()-startTime)<turnTime):
        # print("turning left")
        #turn left
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 1
        velocity_cmd.publish(vel_msg)
        


def main(args):
    #rospy.init_node('robot_controller', anonymous=True)
    rc = robot_controller()
    # try:
    rospy.spin()
    # except KeyboardInterrupt:
        # print("Shutting down")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)
    # initializeAfterSpawn()
    main(sys.argv)












