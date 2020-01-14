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
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class hough_lines:
 
  def __init__(self):
    #-- Create a publisher in topic "image_hough" and "nav_hough_lines"
    self.nav_hough_lines_pub = rospy.Publisher("bebop/nav_hough_lines",Twist, queue_size = 100)
    #self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size = 100)

    #-- Create a supscriber from topic "image_raw"
    self.image_sub = rospy.Subscriber("/bebop/image_raw/compressed", CompressedImage, self.callback, queue_size = 100)

###############################################################################
   
  def callback(self,data):
    global list_hough, pidTerm, last_error1, int_error1

    numLines=3
    yaw = 0
    x = 0
    med_theta = 0
    lines_vector = [0, 0, 0]

    np_arr = np.fromstring(data.data, np.uint8)
    #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    (rows,cols,channels) = image_np.shape
    rospy.loginfo("rows: %f",rows)
    rospy.loginfo("cols: %f",cols)
    rospy.loginfo("-------------------------")


    #-- Convert in gray scale
    gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #Deteccao de bordas
    edges = cv2.Canny(gray, 300, 350, apertureSize=3, L2gradient=True) 

    #Deteccao de linhas
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
   
                        cv2.line(image_np, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        med_theta = med_theta + (theta/numLines)
                        lines_vector[i] = theta
                        x = x+x1+x2

    mediana = int(x/(numLines*2))

    med_theta = math.degrees(med_theta)
    
    # zerar erro de leitura Yaw
    if abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[1]) ) < 60 and abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[2]) ) < 60 and abs( math.degrees(lines_vector[1]) - math.degrees(lines_vector[2]) ) < 60:
      if med_theta > (90):
        yaw = (180-med_theta)
      else:
        yaw = -med_theta

    # rospy.loginfo("linha 1: %f",math.degrees(lines_vector[0]))
    # rospy.loginfo("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.loginfo("linha 3: %f",math.degrees(lines_vector[2]))

    # rospy.loginfo("Media Theta: %f",med_theta)
    # rospy.loginfo("Valor x: %f",x)
    # rospy.loginfo("-------------------------")

    ganho_pid = 1000
    # y in the drone of ROS = X in the image
    y_correction = float(mediana - gray.shape[1]/2)/ganho_pid

    #rospy.loginfo("half_img: %f",gray.shape[1]/2)
    #rospy.loginfo("mediana: %f",mediana)
    rospy.loginfo("y(raw): %f",y_correction)

    rospy.loginfo("yaw(raw): %f",yaw)
    rospy.loginfo("-------------------------")

    # rospy.loginfo("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.loginfo("linha 3: %f",math.degrees(lines_vector[2]))


    nav_drone = Twist()

    if lines is not None:
      nav_drone.linear.x = 0
      nav_drone.linear.y = y_correction
      nav_drone.linear.z = 0

      nav_drone.angular.x = 0
      nav_drone.angular.y = 0
      nav_drone.angular.z = yaw*(np.pi/180)
    else:
      nav_drone.linear.x = 0
      nav_drone.linear.y = 0
      nav_drone.linear.z = 0

      nav_drone.angular.x = 0
      nav_drone.angular.y = 0
      nav_drone.angular.z = 0

    try:
      self.nav_hough_lines_pub.publish(nav_drone)
      #rospy.loginfo('Is publish!')
    except:
      rospy.loginfo('No publish!')

    #cv2.imshow("Image",src_image)
    #cv2.imshow("Image-edges",edges)
    #cv2.waitKey(1)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    # Publish new image
    # try:
    #   self.image_pub.publish(msg)
    #   #rospy.loginfo('Is publish img!')
    # except:
    #   rospy.loginfo('No publish img!')

    # try:
    #   self.vel_drone_pub.publish(self.bridge.cv2_to_imgmsg(src_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)


###############################################################################

def main(args):

  ic = hough_lines()
  #-- Name of node
  rospy.init_node('hough',log_level=rospy.DEBUG)

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)