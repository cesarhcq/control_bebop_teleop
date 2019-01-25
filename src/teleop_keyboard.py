#!/usr/bin/env python
import roslib; roslib.load_manifest('control_bebop_teleop')
import rospy

from geometry_msgs.msg import Twist # Geometry msg is informed by autonomy ROS package (Twitst - Keyboard)
from std_msgs.msg import Empty # std_msg.msg is also informed by autonomy ROS package (Empty is a message to input the control of the quadcopter)

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

't : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

----------------------------------------------------------
TakeOff            - Press 1
Landing            - Press 2
MoveCameraDown     - Press 3
MoveCameraForward  - Press 4
----------------------------------------------------------
CTRL-C to quit
"""
# Bindings values using a mapping keyboard for each key used to move the quadcopter
moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }
# Speed bindings to increase/decrease the velocity of the quadcopter
speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

###############################################################################

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

###############################################################################

def moveCameraDown():

  cam_twist = Twist()

  #-- set camera, look to dwn
  cam_twist.angular.x = 0
  cam_twist.angular.y = -84 #-84 to 15
  cam_twist.angular.z = 0 #-35 to 35 
  cam_pub.publish(cam_twist)
  print('angle_camera: ',cam_twist.angular.y)

  return cam_twist.angular.y

###############################################################################

def moveCameraForward():

  cam_twist = Twist()

  #-- set camera, look to dwn
  cam_twist.angular.x = 0
  cam_twist.angular.y = 0 #-84 to 15
  cam_twist.angular.z = 0 #-35 to 35 
  cam_pub.publish(cam_twist)
  print('angle_camera: ',cam_twist.angular.y)

  return cam_twist.angular.y

###############################################################################
if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
	pub2 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1) # add a publisher for each new topic
	pub3 = rospy.Publisher('bebop/land', Empty, queue_size = 1)    # add a publisher for each new topic
	cam_pub = rospy.Publisher("bebop/camera_control",Twist, queue_size = 1)
	empty_msg = Empty() 
	rospy.init_node('teleop_keyboard')

	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			print 'key'  # add a print for each key pressed
			print  key
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15

			elif key == '1': # condition created in order to pressed key 1 and generates the take off of the bebop2
				print 'key 1 pressionado'
				pub2.publish(empty_msg) # action to publish it

			elif key == '2': # condition created in order to pressed key 2 and generates the land of the bebop2
				print 'key 2 pressionado'
				pub3.publish(empty_msg)	# action to publish it
			elif key == '3': # condition created in order to pressed key 3 and generates the camera look down
				print 'key 3 pressionado'
				moveCameraDown()
			elif key == '4': # condition created in order to pressed key 3 and generates the camera look forward
				print 'key 4 pressionado'
				moveCameraForward()
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


