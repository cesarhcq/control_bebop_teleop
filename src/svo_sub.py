#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import sys, time, math
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
    
###############################################################################
 
class svo_data:
 
  def __init__(self):  

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.svo_image_sub = rospy.Subscriber("svo/image",Image,self.callback)

###############################################################################
   
  def callback(self,data):

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    cv2.imshow("Image-SVO", src_image)
    #cv2.imshow("Image-Gray", gray)
    cv2.waitKey(1)


###############################################################################

def main(args):

  ic = svo_data()
  #-- Name of node
  rospy.init_node('svo_data')

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)