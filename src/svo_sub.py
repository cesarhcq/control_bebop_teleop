#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import sys, time, math
from math import sin, cos, pi

import rospy
import cv2, tf

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


###############################################################################
 
class svo_data:
 
  def __init__(self):  

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.svo_image_sub = rospy.Subscriber("svo/image",Image,self.callback)

    self.svo_pose_sub = rospy.Subscriber("svo/pose",PoseWithCovarianceStamped,self.PoseCallback)
    self.svo_pose_pub = rospy.Publisher("svo/pose",PoseWithCovarianceStamped,queue_size=10)

  ###############################################################################
   
  def callback(self,imagedata):

    try:
      src_image = self.bridge.imgmsg_to_cv2(imagedata, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    #cv2.imshow("Image-SVO", src_image)
    #cv2.imshow("Image-Gray", gray)
    cv2.waitKey(1)

  ###############################################################################

  def PoseCallback(self,posedata):

    robot_pose = PoseWithCovarianceStamped() # [time, [x,y,yaw]]
    tf_br = tf.TransformBroadcaster()

    robot_pose.header = posedata.header
    robot_pose.pose = posedata.pose

    # if(robot_pose.header != None):
    #   print('robot position update!')
    #   euler = tf.transformations.euler_from_quaternion([robot_pose.pose.pose.orientation.x, 
    #                                                     robot_pose.pose.pose.orientation.y, 
    #                                                     robot_pose.pose.pose.orientation.z, 
    #                                                     robot_pose.pose.pose.orientation.w]) #roll, pitch, yaw

    #   print(" x: {}\n y: {}\n Yaw: {}\n".format(robot_pose.pose.pose.position.x*20, robot_pose.pose.pose.position.y*20, euler[2])) # in radians
    #   print('-------------------------------')

    tf_br.sendTransform((robot_pose.pose.pose.position.x*20, robot_pose.pose.pose.position.y*20, robot_pose.pose.pose.position.z*20), 
                          *robot_pose.pose.pose.orientation, 
                          robot_pose.header.stamp, 
                          "svo_base_link", 
                          "aruco_base")

    print(robot_pose)

    try:
      self.svo_pose_pub.publish(robot_pose)
    except:
      print('No publish!')


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