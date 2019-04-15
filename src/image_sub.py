#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import sys, time, math
from math import sin, cos, pi
import rospy
import cv2, tf

# numpy and scipy
import numpy as np
import cv2.aruco as aruco

# library to get image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# library use pose mensages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#-- Define Tag\n",
id_to_find = 273 # 1 273
marker_size = 0.5 # 0.7 #-m -  0.172 m 

#-- Define the Aruco dictionary\n",
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) #pata de urso
#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50) #
parameters =  aruco.DetectorParameters_create()

#-- Get the camera calibration\n",
calib_path = '/home/victor/bebop_ws/src/control_bebop_teleop/'
camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter = ',')

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

first_time = 0

###############################################################################
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
  #-- transpose the matrix R
  Rt = np.transpose(R)

  #-- verify if Rt could be identity
  shouldBeIdentity = np.dot(Rt, R)

  #-- create a identity
  I = np.identity(3, dtype=R.dtype)
  n = np.linalg.norm(I - shouldBeIdentity)

  return n < 1e-6

###############################################################################

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
    
###############################################################################
 
class aruco_odom:
 
  def __init__(self):

    #-- Create a publisher to topic "aruco_results"
    self.pose_aruco_pub = rospy.Publisher("bebop/pose_aruco",Odometry, queue_size = 100)
    self.image_pub = rospy.Publisher("bebop/image_aruco",Image, queue_size = 100)
    #self.orientation_euler_pub = rospy.Publisher("bebop/orientation_euler",Twist, queue_size = 100)

    #-- Create a supscriber from topic "image_raw" and publisher to "bebop/image_aruco"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw",Image,self.callbackImage)

    self.Keyframe_aruco = 0

###############################################################################
   
  def callbackImage(self,data):

    global first_time

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      (rows,cols,channels) = src_image.shape
    except CvBridgeError as e:
      print(e)

    #-- 180 deg rotation matrix around x axis
    R_flip = np.zeros((3,3), dtype=np.float)
    R_flip[0,0] = +1.0
    R_flip[1,1] = -1.0
    R_flip[2,2] = -1.0

    #-- Convert in gray scale\n",
    gray = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image\n",
    corners, ids, rejected = aruco.detectMarkers(image=gray,
                                                  dictionary=aruco_dict,
                                                  parameters=parameters,
                                                  cameraMatrix=camera_matrix,
                                                  distCoeff=camera_distortion)

    if ids[0] == id_to_find:
      #-- ret= [rvec,tvec, ?]
      #-- array of rotation and position of each marker in camera frame
      #-- rvec = [[rvec_1, [rvec2], ...]]  attitude of the marker respect to camera frame
      #-- tvec = [[tvec_1, [tvec2], ...]]  position of the marker in camera frame
      ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

      #-- Unpack the output, get only the first\n",
      rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

      #-- Draw the detected marker and put a reference frame over it\n",
      aruco.drawDetectedMarkers(src_image, corners)
      aruco.drawAxis(src_image, camera_matrix, camera_distortion, rvec, tvec, 0.3)

      #-- Obtain the rotation matrix tag->camera
      R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
      R_tc = R_ct.T # function transpose() with '.T'

      #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
      pitch_marker, roll_marker, yaw_marker = rotationMatrixToEulerAngles(R_tc)

      #-- Now get Position and attitude f the camera respect to the marker
      #pos_camera = -R_tc*np.matrix(tvec).T
      pitch_camera, roll_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

      pos_camera = Point(-tvec[0], tvec[1], tvec[2])

      #-- Print 'X' in the center of the camera
      cv2.putText(src_image, "X", (cols/2, rows/2), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

      ###############################################################################

      #-- Print the tag position in camera frame
      str_position = "Position x = %4.0f  y = %4.0f  z = %4.0f"%(pos_camera.x, pos_camera.y, pos_camera.z)
      cv2.putText(src_image, str_position, (0, 30), font, 2, (255, 255, 0), 2, cv2.LINE_AA)

      #-- Get the attitude of the camera respect to the frame
      str_attitude = "Attitude pitch = %4.0f  roll = %4.0f  yaw = %4.0f"%(math.degrees(0),math.degrees(0),
                          math.degrees(yaw_camera))
      cv2.putText(src_image, str_attitude, (0, 60), font, 2, (255, 255, 0), 2, cv2.LINE_AA)

      ###############################################################################

      # cv2.imshow("Image-Aruco", src_image)
      cv2.waitKey(1)

      aruco_odom = Odometry()
      #aruco_odom.header.stamp = rospy.Time.now()-first_time
      aruco_odom.header.stamp = rospy.Time.now()
      aruco_odom.header.frame_id = "odom_aruco"
      aruco_odom.header.seq = self.Keyframe_aruco
      aruco_odom.child_frame_id = "drone_base"

      # since all odometry is 6DOF we'll need a quaternion created from yaw
      odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_camera)

      # set the position
      aruco_odom.pose.pose = Pose(pos_camera, Quaternion(*odom_quat))

      tf_br = tf.TransformBroadcaster()
      tf_br.sendTransform((pos_camera.x, pos_camera.y, pos_camera.z), 
                          odom_quat, 
                          aruco_odom.header.stamp, 
                          "drone_base",
                          "odom_aruco") #world

      euler_ori = Twist()
      euler_ori.linear.x = -tvec[0]
      euler_ori.linear.y = tvec[1]
      euler_ori.linear.z = tvec[2]

      euler_ori.angular.x = math.degrees(0)
      euler_ori.angular.y = math.degrees(0)
      euler_ori.angular.z = math.degrees(yaw_camera)

      self.Keyframe_aruco += 1

      try:
        self.pose_aruco_pub.publish(aruco_odom)
      except:
        print('No publish!')

    else:
      rospy.loginfo('No Id detected!')
      #print('No Id detected!')

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

###############################################################################

def main(args):
  global first_time

  ic = aruco_odom()
  #-- Name of node
  rospy.init_node('aruco_odom')
  
  rospy.loginfo('init_node')
  first_time = rospy.Time.now()

  try: 
    rospy.spin()
  except rospy.ROSInterruptException:
    print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)