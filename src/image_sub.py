#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import sys, time, math
import rospy
import cv2

# numpy and scipy
import numpy as np
import cv2.aruco as aruco

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

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

#-- Update fps
def update_fps_read():
    global t_read, fps_read
    t           = time.time()
    fps_read    = 1.0/(t - t_read)
    t_read      = t
    
def update_fps_detect():
    global t_detect, fps_detect
    t           = time.time()
    fps_detect  = 1.0/(t - t_detect)
    t_detect      = t
    
t_read      = time.time()
t_detect    = t_read
fps_read    = 0.0
fps_detect  = 0.0
    
###############################################################################
 
class aruco_data:
 
  def __init__(self):
    #self.image_pub = rospy.Publisher("bebop2/camera_base/image_aruco",Image, queue_size=10)

    #-- Create a publisher to topic "aruco_results"
    self.pose_pub = rospy.Publisher("bebop/aruco_results",Twist, queue_size=10)    
    
    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw",Image,self.callback)

###############################################################################
   
  def callback(self,data):

    #-- Define Tag\n",
    id_to_find = 1
    marker_size = 17.2 #-cm

    #-- Define the Aruco dictionary\n",
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters =  aruco.DetectorParameters_create()

    #-- Get the camera calibration\n",
    calib_path = '/home/cesar/bebop_ws/src/control_bebop_teleop/'
    camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter = ',')
    camera_distortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter = ',')

    #-- 180 deg rotation matrix around x axis
    R_flip = np.zeros((3,3), dtype=np.float)
    R_flip[0,0] = +1.0
    R_flip[1,1] = -1.0
    R_flip[2,2] = -1.0

    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    try:
      src_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #-- Convert in gray scale\n",
    gray = cv2.cvtColor(src_image, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    (rows,cols,channels) = src_image.shape
    #if cols > 60 and rows > 60 :
      #cv2.circle(src_image, (50,50), 10, 255)
  
    #print('cols = {} rows = {}'.format(cols,rows))

    #-- Find all the aruco markers in the image\n",
    corners, ids, rejected = aruco.detectMarkers(image=gray,
                                                  dictionary=aruco_dict,
                                                  parameters=parameters,
                                                  cameraMatrix=camera_matrix,
                                                  distCoeff=camera_distortion)

    if ids != None and ids[0] == id_to_find:
      #-- ret= [rvec,tvec, ?]
      #-- array of rotation and position of each marker in camera frame
      #-- rvec = [[rvec_1, [rvec2], ...]]  attitude of the marker respect to camera frame
      #-- tvec = [[tvec_1, [tvec2], ...]]  position of the marker in camera frame
      ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

      #-- Unpack the output, get only the first\n",
      rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

      #-- Draw the detected marker and put a reference frame over it\n",
      aruco.drawDetectedMarkers(src_image, corners)
      aruco.drawAxis(src_image, camera_matrix, camera_distortion, rvec, tvec, 15)

      #-- Obtain the rotation matrix tag->camera
      R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
      R_tc = R_ct.T # function transpose() with '.T'

      #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
      roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_tc)

      #-- Now get Position and attitude f the camera respect to the marker
      pos_camera = -R_tc*np.matrix(tvec).T
      roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

      #-- Print 'X' in the center of the camera
      cv2.putText(src_image, "X", (cols/2, rows/2), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
      ###############################################################################

      #-- Print the tag position in camera frame
      str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f "%(tvec[0], tvec[1], tvec[2])
      cv2.putText(src_image, str_position, (0, 20), font, 1, (255, 255, 0), 1, cv2.LINE_AA)
      
      #-- Print the marker's attitude respect to camera frame
      str_attitude = "MARKER Attitude p=%4.0f  r=%4.0f  y=%4.0f"%(math.degrees(pitch_marker), math.degrees(roll_marker),
                          math.degrees(yaw_marker))
      cv2.putText(src_image, str_attitude, (0, 40), font, 1, (255, 255, 0), 1, cv2.LINE_AA)

      ###############################################################################

      #-- Print the tag position in camera frame
      str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(-tvec[0], tvec[1], tvec[2])
      cv2.putText(src_image, str_position, (0, 70), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

      #-- Get the attitude of the camera respect to the frame
      str_attitude = "CAMERA Attitude p=%4.0f  r=%4.0f  y=%4.0f"%(math.degrees(pitch_camera),math.degrees(roll_camera),
                          math.degrees(yaw_camera))
      cv2.putText(src_image, str_attitude, (0, 90), font, 1, (255, 255, 255), 1, cv2.LINE_AA)

      ###############################################################################
      
      cv2.imshow("Image-Aruco", src_image)
      #cv2.imshow("Image-Gray", gray)
      cv2.waitKey(2)

      twist = Twist()
      twist.linear.x = -tvec[0]
      twist.linear.y = tvec[1]
      twist.linear.z = tvec[2]

      twist.angular.x = math.degrees(pitch_camera)
      twist.angular.y = math.degrees(roll_camera)
      twist.angular.z = math.degrees(yaw_camera)

    else:
      print('Aruco not detected!')
      #-- Display the resulting frame\n",
      cv2.imshow("Image-Aruco",src_image)
      cv2.waitKey(2)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

    #-- Publish the pose of marker of aruco to topics
    try:
      self.pose_pub.publish(twist)
    except:
      print('Can not find aruco marker!')

###############################################################################

def main(args):

  ic = aruco_data()
  #-- Name of node
  rospy.init_node('aruco_data', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
       main(sys.argv)