#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import time, math
import sys, select, termios, tty
import rospy
import tf
import cv2

# numpy and scipy
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

list_hough = []

pidTerm = 0
last_error1 = 0
int_error1 = 0

drone_pose = Odometry()

vel_hough = Twist()

msg_aruco = "Empty"

landing = True

navigation = True

msg = """
Keyboard commands for Autonomous Landing of the Quadcopter
----------------------------------------------------------
TakeOff         - Press 1
Landing         - Press 2
MoveCamera      - Press 3
MoveUp          - Press 4
MoveDown        - Press 5
Auto-Landing    - Press 6
Auto-Navigation - Press 7
----------------------------------------------------------
Ctrl + C to quit
"""

###############################################################################

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

###############################################################################

def moveCamera():

  cam_twist = Twist()

  #-- set camera, look to dwn
  cam_twist.angular.x = 0
  cam_twist.angular.y = -84
  cam_twist.angular.z = 0
  cam_pub.publish(cam_twist)
  print('angle_camera: ',cam_twist.angular.y)

  return cam_twist.angular.y

###############################################################################

def moveUp():
  global drone_pose

  velocity = Twist()

  cont = 0

  while not rospy.is_shutdown() and cont < 500:

    #print('init cont: ', cont)
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 1 # velocity.linear.z = 1 para subir


    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    vel_drone_pub.publish(velocity)

    cont += 1
    print('Z with Aruco:',drone_pose.pose.pose.position.z)

    # print('velocity-linear-X: {} - velocity-linear-Y: {} - velocity-linear-Z: {}'.format(velocity.linear.x, velocity.linear.y, velocity.linear.z))
    rate.sleep()

 ###############################################################################

def moveDown():
  global drone_pose
  
  velocity = Twist()

  cont = 0

  while not rospy.is_shutdown() and cont < 500:

    #print('init cont: ', cont)
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = -1

    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0
    vel_drone_pub.publish(velocity)

    cont += 1
    print('Z with Aruco:',drone_pose.pose.pose.position.z)

    rate.sleep()

###############################################################################

def autoNavigation():
    global list_hough, pidTerm, last_error1, int_error1, vel_hough 

    med_hough = 0
    yaw = math.degrees(vel_hough.angular.z)
    rospy.loginfo('Yaw (Raw): %f deg/s',yaw)

    while not rospy.is_shutdown() and navigation:
      # Filter
      if len(list_hough) < 20:
          list_hough.append(yaw)
          
      else:
          list_hough.append(yaw)
          del list_hough[0]
          med_hough = sum(list_hough)/len(list_hough)
          #rospy.loginfo('Size of list_hough %f',len(list_hough))
          rospy.loginfo('Yaw (Filter): %f deg/s',med_hough)
          rospy.loginfo("-------------------------")

      #Gain controll
      Kp = 0.5  #2.0
      Ki = 0.001  #0.1
      #Kd = 0.1  #0.5

      #Erro of Yaw
      erro_yaw = med_hough
      if(abs(erro_yaw) > 3):
          pidTerm = Kp*erro_yaw + Ki*int_error1 #+ Kd*(erro_yaw-last_error1)
          int_error1 += erro_yaw
          #last_error1 = erro_yaw
          new_yaw = pidTerm
          rospy.loginfo('new_yaw: %f',new_yaw)
          rospy.loginfo("-------------------------")
      else:
          new_yaw = 0
          int_error1 = 0
          #last_error1 = 0
          rospy.loginfo('new_yaw: %f',new_yaw)
          rospy.loginfo("-------------------------")
      
      velocity = Twist()


      #rospy.loginfo('------------------Init Navigation----------------------')
      velocity.linear.x = 0
      velocity.linear.y = 0 #-vel_hough.linear.y
      velocity.linear.z = 0
      
      # rospy.loginfo('vel_linear x: %f', vel_hough.linear.x)
      # rospy.loginfo('vel_linear y: %f', -vel_hough.linear.y)
      # rospy.loginfo('vel_linear z: %f', vel_hough.linear.z)

      velocity.angular.x = 0
      velocity.angular.y = 0
      velocity.angular.z = new_yaw*(np.pi/180)

      vel_drone_pub.publish(velocity)

      rate.sleep()


def callbackNavHough(posedata):

  global vel_hough

  vel_hough = posedata

  ###############################################################################
   
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('autonomous_navigation',log_level=rospy.DEBUG)

  # create the important subscribers
  hough_sub = rospy.Subscriber("bebop/nav_hough_lines",Twist, callbackNavHough, queue_size = 100)

  # create the important publishers
  cam_pub = rospy.Publisher("bebop/camera_control",Twist, queue_size = 100)
  vel_drone_pub = rospy.Publisher("bebop/cmd_vel",Twist, queue_size = 100)

  # create the publishers to take off and land
  takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size = 100) # add a publisher for each new topic
  land_pub = rospy.Publisher('bebop/land', Empty, queue_size = 100)    # add a publisher for each new topic
  
  empty_msg = Empty() 

  rate = rospy.Rate(100.0) #-- 100Hz

  print('Program Started')
  print(msg)

  try:
    while(True):
      key = getKey()
      print('key')  # add a print for each key pressed
      print(key)

      if key == '1': # condition created in order to pressed key 1 and generates the take off of the bebop2
        print('key 1 pressed - Takeoff')
        takeoff_pub.publish(empty_msg) # action to publish it
        print(msg)

      elif key == '2': # condition created in order to pressed key 2 and generates the land of the bebop2
        print('key 2 pressed - Landing')
        land_pub.publish(empty_msg) # action to publish it
        print(msg)

      elif key == '3': # condition created in order to pressed key 3 and generates the moviment of camera
        print('key 3 pressed - MoveCamera')
        moveCamera()
        print(msg)

      elif key == '4': # condition created in order to pressed key 4 and generates upward movement of the drone
        print('key 4 pressed - MoveUp')
        moveUp()
        print(msg)

      elif key == '5': # condition created in order to pressed key 5 and generates drone descent movement
        print('key 5 pressed - MoveDown')
        moveDown()
        print(msg)

      elif key == '6': # condition created in order to pressed key 6 and generates Auto-Landing of drone
        print('key 6 pressed - Autonomous-Navigation') 
        autoNavigation()
        print(msg)

      elif key == '\x03': # condition created in order to pressed key Ctrl+c and generates output from the program
          print('Quit work')
          break
      else:
        print('Wrong key!')
        print(msg)

      navigation = True
      landing = True

  except rospy.ROSInterruptException:
    print('Erro')
  finally:
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)