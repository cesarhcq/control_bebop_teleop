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

first_flight = True
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

  #-- set camera, look to down
  cam_twist.angular.x = 0
  cam_twist.angular.y = -90
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

  while cont < 200:

    print('init cont: ', cont)
    first_position.linear.x = 0
    first_position.linear.y = 0
    first_position.linear.z = 1 # first_position.linear.z = 1 para subir


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

  while cont < 100:

    print('init cont: ', cont)
    first_position.linear.x = 0
    first_position.linear.y = 0
    first_position.linear.z = -1

    first_position.angular.x = 0
    first_position.angular.y = 0
    first_position.angular.z = 0
    pose_pub.publish(first_position)

    cont += 1

    print('X: {} - Y: {} - Z: {}'.format(first_position.linear.x, first_position.linear.y, first_position.linear.z))
    rate.sleep()

  ###############################################################################

def move2Aruco():
  global msg_aruco

  # z data orientation
  k=0.005
  ki=0.005
  eyawp = 0

  # x data translation

  k_x = 0.001
  k_i_x = 0.0003
  exp = 0

  # y data translation

  k_y = 0.001
  k_i_y = 0.0003
  eyp = 0

  goal_aruco = Twist()
  empty_msg = Empty()

  angle_camera = moveCamera()
  
  while not rospy.is_shutdown():

    if msg_aruco == "Aruco Found!" and angle_camera == -90:
      # Condition for translation in Yaw

      if abs(angularz) > 2:
        uyaw = k*angularz+(angularz+eyawp)*ki
        eyawp = angularz
        print('correcting rotation Yaw')
      else:
        uyaw = 0
        print('Yaw close to 0')

      # Condition for translation in X = 15

      if abs(linearx) > 10 and abs(angularz) <= 2: # x=8
        u_x = k_x*linearx + (linearx+exp)*k_i_x
        exp = linearx
        print('correcting translation X')
      else:
        u_x = 0
        print('X close to 0')

      # Condition for translation in y

      if abs(lineary) > 8 and abs(angularz) <= 2: # y=6
        u_y = k_y*lineary + (lineary+eyp)*k_i_y
        eyp = lineary
        u_y = u_y # ---????
        print('correcting translation Y')
      else:
        u_y = 0
        print('Y close to 0')

      # Condition for translation in z

      if abs(linearz) > 180 and abs(lineary) <= 8 and abs(linearx) <= 10 and abs(angularz) <= 2:
        u_z = -0.5
        print('correcting translation Z')
      else:
        u_z = 0
        print('Z close to 0')

      # Condition landing

      # if abs(linearz) <= 160 and abs(lineary) <= 10 and abs(linearx) <= 8 and abs(angularz) <= 2:
      #   goal_aruco.linear.x = -10
      #   pose_pub.publish(goal_aruco)
      #   land_pub.publish(empty_msg)
      #   print('Auto-Landing Performed2!')

      print('RegularX: {} - RegularY: {} - RegularYaw: {}'.format(u_x, u_y, uyaw))

      goal_aruco.linear.y = u_x
      goal_aruco.linear.x = -u_y
      goal_aruco.linear.z = u_z

      goal_aruco.angular.z = uyaw

      pose_pub.publish(goal_aruco)

      if abs(linearz) <= 180 and abs(lineary) <= 8 and abs(linearx) <= 10 and abs(angularz) <= 2:
        goal_aruco.linear.x = 1.0
        pose_pub.publish(goal_aruco)
        land_pub.publish(empty_msg)
        print('Auto-Landing Performed')

    # else:
    #   print('Auto-Landing not Performed!')
    #   goal_aruco.linear.y = 0
    #   goal_aruco.linear.x = 0
    #   goal_aruco.linear.z = 0

    #   goal_aruco.angular.x = 0
    #   goal_aruco.angular.y = 0
    #   goal_aruco.angular.z = 0
    #   pose_pub.publish(goal_aruco)

    rate.sleep()

###############################################################################
   
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('landing_aruco')

  pose_sub = rospy.Subscriber("bebop/aruco_results",Twist, callback)
  msg_sub = rospy.Subscriber("bebop/aruco_data_received",String, callback_msg)


  cam_pub = rospy.Publisher("bebop/camera_control",Twist, queue_size=10)
  pose_pub = rospy.Publisher("bebop/cmd_vel",Twist, queue_size=10)



  takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size = 10) # add a publisher for each new topic
  land_pub = rospy.Publisher('bebop/land', Empty, queue_size = 10)    # add a publisher for each new topic
  empty_msg = Empty() 

  rate = rospy.Rate(10) #-- 10Hz

  print('Program Started')
  print(msg)

  first_flight = True

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

      elif key == '3': # condition created in order to pressed key 3 and generates the land of the bebop2
        print('key 3 pressed - MoveCamera')
        moveCamera()

      elif key == '4': # condition created in order to pressed key 3 and generates the land of the bebop2
        print('key 4 pressed - MoveUp')
        moveUp()

      elif key == '5': # condition created in order to pressed key 4 and generates the land of the bebop2
        print('key 5 pressed - MoveDown')
        moveDown()

      elif key == '6':
        print('key 6 pressed - Auto-Landing') 
        move2Aruco()

      elif key == '\x03':
          print('Quit work')
          break

      else:
        print('Wrong key!')
      
  except rospy.ROSInterruptException:
    print('Erro')