#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import sys, time, math
import rospy
import cv2

# numpy and scipy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import String

 
class hough_lines:
 
  def __init__(self):
    self.image_pub = rospy.Publisher("bebop/image_hough",Image, queue_size=100) 
    
    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw",Image,self.callback)

###############################################################################
   
  def callback(self,data):
    numLines=3 

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #-- Convert in gray scale\n",
    gray = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    edges = cv2.Canny(gray, 50, 50, apertureSize=3, L2gradient=True) #Deteccao de bordas

    lines = cv2.HoughLines(edges, numLines, np.pi/90, 100)

    if lines is not None: 
            if lines.shape[0] >= numLines:
                x = 0
                med_theta = 0
                for i in range(0,numLines):
                    for rho, theta in lines[i]:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
   
                        cv2.line(src_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        med_theta = med_theta + (theta/numLines)
                        x = x+x1+x2 
    

    cv2.imshow("Image",src_image)
    #cv2.imshow("Image-edges",edges)
    cv2.waitKey(1)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


###############################################################################

def main(args):

  ic = hough_lines()
  #-- Name of node
  rospy.init_node('hough')

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)