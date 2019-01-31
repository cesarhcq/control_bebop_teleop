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


def handle_aruco_pose(pose_msg, aruco_tag):
    odom_broadcaster = tf.TransformBroadcaster()

    odom_broadcaster.sendTransform((pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z),
                     (pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     aruco_tag,
                     "world")

def publisher_of_tf():

    rospy.init_node('publisher_of_tf_node')

    time,sleep(1)
    rospy.loginfo('Ready..publishing tf now...')

    rate = rospy.Rate(5.0) #-- 5Hz

    while not rospy.is_shutdown():

      if not pose_now:
        print "no pose"
      else:
        handle_aruco_pose(pose_now,aruco_tag)
      rate.sleep()


###############################################################################
   
if __name__ == '__main__':
    try:
        publisher_of_tf()
    except rospy.ROSInterruptException:
        pass