#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *



import sys
import os
import signal
import subprocess
import numpy as np



class GPSToLocalizationPose:

    def __init__(self):
      self.pub_ndt_pose = rospy.Publisher('/localization_pose', PoseStamped, queue_size=10)
      self.sub_gps_pose = rospy.Subscriber('/gps_local/pose', PoseStamped, self.gpsPoseCallBack)
      self.x_y_sigma = rospy.get_param('~pose_stddev_x_y', 0.0)
      self.x_y_mu = rospy.get_param('~pose_mu_x_y', 0.0)

      self.yaw_sigma = rospy.get_param('~pose_stddev_yaw', 0.0)
      self.yaw_mu = rospy.get_param('~pose_mu_yaw', 0.0)

      self.x_noise = np.random.normal(self.x_y_mu, self.x_y_sigma, size=200)
      self.y_noise = np.random.normal(self.x_y_mu, self.x_y_sigma, size=200)
      self.yaw_noise = np.random.normal(self.yaw_mu, self.yaw_sigma, size=200)

    def gpsPoseCallBack(self, data):

      yaw_noise = np.random.normal(self.yaw_mu, self.yaw_sigma, size=1)

      indx = (data.header.seq % len(self.x_noise)-1)
      data.pose.position.x = data.pose.position.x + self.x_noise[indx]
      data.pose.position.y = data.pose.position.y + self.y_noise[indx]


      q_orig = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

      q_rot = quaternion_from_euler(0, 0, yaw_noise)
      q_new = quaternion_multiply(q_orig, q_rot)
      data.pose.orientation.x = q_new[0]
      data.pose.orientation.y = q_new[1]
      data.pose.orientation.z = q_new[2]
      data.pose.orientation.w = q_new[3]

      self.pub_ndt_pose.publish(data)


if __name__ == '__main__':
    try:
        rospy.init_node('gps_to_localization_pose') # Initialize ROS
        GPSToLocalizationPose()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])

