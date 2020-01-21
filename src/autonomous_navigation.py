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
from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

list_hough = []

last_error_yaw = 0
int_error_yaw = 0

last_error_y = 0
int_error_y = 0

drone_pose = Odometry()
odom = Odometry()
vel_hough = Twist()
class_rnn = Vector3()

msg_aruco = "Empty"

landing = True
navigation = True
init = True

msg = """
Keyboard commands for Autonomous Landing of the Quadcopter
----------------------------------------------------------
TakeOff         - Press 1
Landing         - Press 2
MoveCamera      - Press 3
MoveUp          - Press 4
MoveDown        - Press 5
Auto-Navigation - Press 6
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

  while not rospy.is_shutdown() and cont < 50:

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

  while not rospy.is_shutdown() and cont < 100:

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
  global vel_hough, class_rnn, list_hough, last_error_yaw, last_error_y, int_error_yaw, int_error_y, odom, init

  med_hough = 0
  pidTerm = 0
  cont = 0

  current_time = rospy.Time.now()
  last_time = rospy.Time.now()

  while not rospy.is_shutdown():

    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    x_raw = vel_hough.linear.x
    y_raw = vel_hough.linear.y
    yaw_raw = math.degrees(vel_hough.angular.z)
    z_raw = odom.pose.pose.position.z
    # reta = 0 / curva = 1
    moviment = class_rnn.x
    rotation = class_rnn.y

    # rospy.loginfo('class_rnn.x: %f',moviment)
    # rospy.loginfo('class_rnn.y: %f',rotation)
    # rospy.loginfo("-------------------------")
    #rospy.loginfo('Y (Raw): %f m/s',y_raw)
    #rospy.loginfo('Z (Raw): %f m/s',z_raw)
    #rospy.loginfo('Yaw (Raw): %f deg/s',yaw_raw)

    # Filter
    if len(list_hough) < 20:
        list_hough.append(yaw_raw)
        
    else:
        list_hough.append(yaw_raw)
        del list_hough[0]
        med_hough = sum(list_hough)/len(list_hough)
        #rospy.loginfo('Size of list_hough %f',len(list_hough))
        # rospy.loginfo('Yaw (Filter): %f deg/s',med_hough)
        # rospy.loginfo("-------------------------")

    #Gain controll Y
    Kpy = 0.00001  #0.0007
    Kiy = 0.000001  #0.000001
    Kdy = 0.0000001  #0.5

    #Erro of Y
    erro_y = y_raw
    if abs(erro_y) > 5:
        pidTerm = Kpy*erro_y + Kiy*int_error_y + Kdy*(erro_y-last_error_y)
        int_error_y += erro_y
        last_error_y = erro_y
        new_y = pidTerm
        #rospy.loginfo('new_y: %f',new_y)
        #rospy.loginfo("-------------------------")
    else:
        new_y = 0
        int_error_y = 0
        last_error_y = 0
        #rospy.loginfo('new_y: %f',new_y)
        #rospy.loginfo("-------------------------")

    kpz = 0.7
    set_point = 1.0
    # y in the drone of ROS = X in the image
    erro_z = float(set_point - z_raw)
    #rospy.loginfo("erro_z %f", erro_z)

    if abs(erro_z) > 0.1:
        new_z = erro_z*kpz
        #rospy.loginfo("Correction Z %f", new_z)
        #rospy.loginfo("-------------------------")
    else:
        new_z = 0
        #rospy.loginfo("Correction Z %f", new_z)
        #rospy.loginfo("-------------------------")

    #Gain controll Yaw
    Kp = 0.6  #2.0
    Ki = 0.001  #0.1
    #Kd = 0.1  #0.5

    #Erro of Yaw
    erro_yaw = med_hough
    if abs(erro_yaw) > 3:
        pidTerm = Kp*erro_yaw + Ki*int_error_yaw #+ Kd*(erro_yaw-last_error_yaw)
        int_error_yaw += erro_yaw
        #last_error_yaw = erro_yaw
        new_yaw = pidTerm
        # rospy.loginfo('new_yaw: %f',new_yaw)
        # rospy.loginfo("-------------------------")
    else:
        new_yaw = 0
        int_error_yaw = 0
        #last_error1 = 0
        # rospy.loginfo('new_yaw: %f',new_yaw)
        # rospy.loginfo("-------------------------")
    
    velocity = Twist()

    # if moviment == 1:
    #     rospy.loginfo("Curva")
    #     # velocity.linear.x = x_raw
    #     # velocity.linear.y = 0
    #     # velocity.linear.z = new_z

    #     # velocity.angular.x = 0
    #     # velocity.angular.y = 0
    #     # velocity.angular.z = 8*(np.pi/180)
    #     if rotation == 1:
    #         velocity.linear.x = 0.02
    #         velocity.linear.y = 0
    #         velocity.linear.z = new_z

    #         velocity.angular.x = 0
    #         velocity.angular.y = 0
    #         velocity.angular.z = 8*(np.pi/180)
    #         #rospy.loginfo("Vel Y: %f m/s",y_correction)
    #         rospy.loginfo("...Left-yaw: %f deg/s",velocity.angular.z*(180/np.pi))
    #         rospy.loginfo("-------------------------")

    #     else:
    #         velocity.linear.x = 0.02
    #         velocity.linear.y = 0
    #         velocity.linear.z = new_z

    #         velocity.angular.x = 0
    #         velocity.angular.y = 0
    #         velocity.angular.z = -8*(np.pi/180)
    #         #rospy.loginfo("Vel Y: %f m/s",y_correction)
    #         rospy.loginfo("...Right-yaw: %f deg/s",velocity.angular.z*(180/np.pi))
    #         rospy.loginfo("-------------------------")
    # else:
          # rospy.loginfo("Reta")
          # velocity.linear.x = 0.015 #0.02
          # velocity.linear.y = -new_y
          # velocity.linear.z = new_z

          # velocity.angular.x = 0
          # velocity.angular.y = 0
          # velocity.angular.z = new_yaw*(np.pi/180)

          # rospy.loginfo('vel_linear  x: %f', 0.015)
          # rospy.loginfo('vel_linear  y: %f',-new_y)
          # rospy.loginfo('vel_linear  z: %f', new_z)
          # rospy.loginfo('vel_angular z: %f', new_yaw)
          # rospy.loginfo("-------------------------")
    if init == True:
      for i in range(50):
        velocity = Twist()
        velocity.linear.y = -0.02
        velocity.linear.z = 2
        vel_drone_pub.publish(velocity)
        rospy.logdebug('Z: SUBINDO!')
        rate.sleep()

    init = False

    #rospy.loginfo("Reta")
    velocity.linear.x = 0
    velocity.linear.y = 0#-new_y
    velocity.linear.z = new_z

    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0#new_yaw*(np.pi/180)

    # rospy.loginfo('vel_linear  x: %f', 0)
    # rospy.loginfo('vel_linear  y: %f', 0)
    rospy.loginfo('vel_linear  z: %f', new_z)
    # rospy.loginfo('vel_angular z: %f', 0)
    rospy.logdebug("-------------------------")

    vel_drone_pub.publish(velocity)

    rate.sleep()

# def callbackNavHough(posedata):
#   global vel_hough

#   vel_hough = posedata

# def callbackRNN(posedata):
#   global class_rnn

#   class_rnn = posedata

def callbackOdom(posedata):
  global odom

  odom = posedata

  ###############################################################################
   
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('autonomous_navigation',log_level=rospy.DEBUG)

  # create the important subscribers
  # hough_sub = rospy.Subscriber("bebop/nav_hough_lines",Twist, callbackNavHough, queue_size = 100)
  # rnn_sub = rospy.Subscriber("bebop/nav_rnn",Vector3, callbackRNN, queue_size = 100)
  #odm_sub = rospy.Subscriber('bebop/odom', Odometry, callbackOdom, queue_size=100)
  rcnn_sub = rospy.Subscriber('rcnn/nav_position', Odometry, callbackOdom, queue_size=100)


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
  # finally:
  #   twist = Twist()
  #   twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
  #   twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
  #   pub.publish(twist)