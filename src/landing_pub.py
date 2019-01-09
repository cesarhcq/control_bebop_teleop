#!/usr/bin/env python
from __future__ import print_function
 
import roslib
roslib.load_manifest('control_bebop_teleop')

import time, math
import sys, select, termios, tty
import rospy
import cv2

# numpy and scipy
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import String

linearx = 0
lineary = 0
linearz = 0

angularx = 0
angulary = 0
angularz = 0

msg_aruco = "Empty"

landing = True
cont = 0

msg = """
Keyboard commands for Autonomous Landing of the Quadcopter
----------------------------------------------------------
TakeOff      - Press 1
Landing      - Press 2
MoveCamera   - Press 3
MoveUp       - Press 4
MoveDown     - Press 5
Auto-Landing - Press 6
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
  print('angle: ',cam_twist.angular.y)

  return cam_twist.angular.y

###############################################################################

def callback(data):
  global linearx, lineary, linearz, angularx, angulary, angularz

  linearx = data.linear.x
  lineary = data.linear.y
  linearz = data.linear.z

  angularx = data.angular.x
  angulary = data.angular.y
  angularz = data.angular.z

###############################################################################

def callback_msg(data2):
  global msg_aruco

  msg_aruco = data2.data

###############################################################################

def moveUp():
  global cont

  first_position = Twist()

  cont = 0

  while cont < 50:

    print('init cont: ', cont)
    first_position.linear.x = 0
    first_position.linear.y = 0
    first_position.linear.z = 0.5 # first_position.linear.z = 1 para subir


    first_position.angular.x = 0
    first_position.angular.y = 0
    first_position.angular.z = 0
    pose_pub.publish(first_position)

    cont += 1

    print('X: {} - Y: {} - Z: {}'.format(first_position.linear.x, first_position.linear.y, first_position.linear.z))
    rate.sleep()

 ###############################################################################

def moveDown():
  global cont
  
  first_position = Twist()

  cont = 0

  while cont < 10:

    print('init cont: ', cont)
    first_position.linear.x = 0
    first_position.linear.y = 0
    first_position.linear.z = -0.5

    first_position.angular.x = 0
    first_position.angular.y = 0
    first_position.angular.z = 0
    pose_pub.publish(first_position)

    cont += 1

    print('X: {} - Y: {} - Z: {}'.format(first_position.linear.x, first_position.linear.y, first_position.linear.z))
    rate.sleep()

  ###############################################################################

def move2Aruco():
  global msg_aruco, lineary, landing

  # Yaw data orientation
  k=0.008
  ki=0.004
  eyawp = 0

  # x data translation

  k_x = 0.001
  k_i_x = 0.0001
  exp = 0

  # y data translation

  k_y = 0.001
  k_i_y = 0.0001
  eyp = 0

  tolerance_Yaw = 2
  tolerance_X = 5
  tolerance_Y = 10

  goal_aruco = Twist()
  empty_msg = Empty()

  angle_camera = moveCamera()
  
  while landing:

    if msg_aruco == "Aruco Found!" and angle_camera == -84:
      
      # Condition for translation in Yaw
      if abs(angularz) > tolerance_Yaw+linearz*0.001:
        uyaw = k*angularz+(angularz+eyawp)*ki
        eyawp = angularz # incrementar
        print('correcting rotation Yaw - ', tolerance_Yaw+linearz*0.001)

      else:
        uyaw = 0
        print('Yaw close to 0')

      # Condition for translation in X
      if abs(linearx) > (tolerance_X+linearz*0.02):
        u_x = k_x*linearx + (linearx+exp)*k_i_x
        exp = linearx
        print('correcting translation X: {} - ux: {}'.format((tolerance_X+linearz*0.02),u_x))
      else:
        u_x = 0
        print('X close to 0')
      
      # Condition for translation in y
      if abs(lineary) > (tolerance_Y+linearz*0.02):
        u_y = k_y*lineary + (lineary+eyp)*k_i_y
        eyp = lineary
        u_y = u_y
        print('correcting translation Y: {} - uy: {}'.format((tolerance_Y+linearz*0.02),u_y))
      else:
        u_y = 0
        print('Y close to 0')


      # Condition for translation in z
      if abs(linearz) > 40 and abs(linearx) <= (tolerance_X+linearz*0.02) and abs(lineary) <= (tolerance_Y+linearz*0.02) and abs(angularz) <= tolerance_Yaw+linearz*0.001:
        u_z = -0.5
        print('correcting translation Z:',linearz)
        #print('RegularX: {} - RegularY: {} - RegularYaw: {}'.format(u_x, u_y, uyaw))
      else:
        u_z = 0
        print('Z close to 0')

      # Command of actuation
      goal_aruco.linear.y = u_x
      goal_aruco.linear.x = -u_y
      goal_aruco.linear.z = u_z

      goal_aruco.angular.x = 0
      goal_aruco.angular.y = 0
      goal_aruco.angular.z = uyaw

      pose_pub.publish(goal_aruco)

      # Condition landing
      if abs(linearz) <= 40 and abs(linearx) <= (tolerance_X+linearz*0.02) and abs(lineary) <= (tolerance_Y+linearz*0.02) and abs(angularz) <= tolerance_Yaw+linearz*0.001:
        goal_aruco.linear.x = 1.0
        pose_pub.publish(goal_aruco)
        land_pub.publish(empty_msg)
        print('Auto-Landing Performed2!')
        landing = False
        break

    else:
      print('Auto-Landing not Performed!')
      goal_aruco.linear.y = 0
      goal_aruco.linear.x = 0
      goal_aruco.linear.z = 0

      goal_aruco.angular.x = 0
      goal_aruco.angular.y = 0
      goal_aruco.angular.z = 0
      pose_pub.publish(goal_aruco)


    print('---------------------------------')
    rate.sleep()

###############################################################################
   
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('landing_aruco')

  pose_sub = rospy.Subscriber("bebop/aruco_results",Twist, callback)
  msg_sub = rospy.Subscriber("bebop/aruco_data_received",String, callback_msg)


  cam_pub = rospy.Publisher("bebop/camera_control",Twist, queue_size=100)
  pose_pub = rospy.Publisher("bebop/cmd_vel",Twist, queue_size=100)


  takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size = 100) # add a publisher for each new topic
  land_pub = rospy.Publisher('bebop/land', Empty, queue_size = 100)    # add a publisher for each new topic
  empty_msg = Empty() 

  rate = rospy.Rate(100) #-- 100Hz

  print('Program Started')
  print(msg)

  try:
    while(1):
      key = getKey()
      print('key')  # add a print for each key pressed
      print(key)

      if key == '1': # condition created in order to pressed key 1 and generates the take off of the bebop2
        print('key 1 pressed - Takeoff')
        takeoff_pub.publish(empty_msg) # action to publish it

      elif key == '2': # condition created in order to pressed key 2 and generates the land of the bebop2
        print('key 2 pressed - Landing')
        land_pub.publish(empty_msg) # action to publish it

      elif key == '3': # condition created in order to pressed key 3 and move camera to land in bebop2
        print('key 3 pressed - MoveCamera')
        moveCamera()

      elif key == '4': # condition created in order to pressed key 4 and move drone up to land in bebop2
        print('key 4 pressed - MoveUp')
        moveUp()

      elif key == '5': # condition created in order to pressed key 5 and move drone down to land in bebop2
        print('key 5 pressed - MoveDown')
        moveDown()

      elif key == '6': # condition created in order to pressed key 6 and auto-landing drone
        print('key 6 pressed - Auto-Landing') 
        move2Aruco()
 
      elif key == '\x03': # condition created in order to pressed key Ctrl+c and quit of work 
          print('Quit work')
          break
      else:
        print('Wrong key!')

      landing = True
      
  except rospy.ROSInterruptException:
    print('Erro')