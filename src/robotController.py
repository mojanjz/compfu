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


        src = np.float32([[0, IMAGE_H], [1207, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        img = cv_image
        img = img[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop
        warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
        cv2.imshow("Image window", warped_img)
        cv2.waitKey(3)
        # color = cv_image[719,1279]
        # print(color)
        # print(warped_img)
        sensitivity = 160 # range of sensitivity=[90,150]
        lower_white = np.array([0,0,255-sensitivity])
        upper_white = np.array([255,sensitivity,255])
        boundaries = [
            # ([17, 15, 100], [50, 56, 200]),
          
              ([0, 200, 0],[255, 255, 255]) #white baby
              ([0, 0, 255-20], [255, 20, 255]), #red
              ([0, 0, 50], [190, 90, 90]) #gray street
              ([10,70,10],[70,210,30]) #grass green

        ]
        # loop over the boundaries
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
        
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(warped_img, lower, upper)
            output = cv2.bitwise_and(warped_img, warped_img, mask = mask)
        
            # show the images
            # cv2.imshow("images", np.hstack([cv_image, output]))
            cv2.imshow("images",output)
            cv2.waitKey(0)
        # plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB)) # Show results
        # plt.show()

        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	    # rows_to_take = 150
	    # cols_to_take = 150
        
        # bottom20 = grayframe[230:]
        # ret, thresh = cv2.threshold(bottom20,127,255,0)
        # img, contours, hierarchy = cv2.findContours(thresh,1,2)

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped_img, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
    def pid(self,centreOfMass,middlePixel):
        zTwist = 0.1
        xVelocity = 0.03
        xDifference = centreOfMass - middlePixel
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












