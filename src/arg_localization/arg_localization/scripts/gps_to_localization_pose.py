#!/usr/bin/env python3

import rospy
import sys

import tf
import tf2_ros
from tf.transformations import *

import geometry_msgs.msg

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

import os
import signal
import subprocess
import numpy as np



class GPSToLocalizationPose:

    def __init__(self):

      self.x_y_sigma = rospy.get_param('~pose_stddev_x_y', 0.0)
      self.x_y_mu = rospy.get_param('~pose_mu_x_y', 0.0)

      self.yaw_sigma = rospy.get_param('~pose_stddev_yaw', 0.0)
      self.yaw_mu = rospy.get_param('~pose_mu_yaw', 0.0)

      self.pub_ndt_pose = rospy.Publisher('/localization/pose_estimator/pose', PoseStamped, queue_size=10)
      self.pub_twist = rospy.Publisher('/localization/pose_estimator/twist', TwistStamped, queue_size=10)

      self.sub_gps_pose = rospy.Subscriber('/gps_local/pose', PoseStamped, self.gpsPoseCallBack)
      self.sub_odom = rospy.Subscriber('/devbot/odom', Odometry, self.odometryCallBack)
      self.alive = 0



    def gpsPoseCallBack(self, data):

      br = tf2_ros.TransformBroadcaster()
      t = geometry_msgs.msg.TransformStamped()

      t.header.stamp = rospy.Time.now()
      t.header.frame_id = "map"
      t.child_frame_id = "base_link"

      t.transform.translation.x = data.pose.position.x
      t.transform.translation.y = data.pose.position.y
      t.transform.translation.z = data.pose.position.z
      
      t.transform.rotation.x = data.pose.orientation.x
      t.transform.rotation.y = data.pose.orientation.y
      t.transform.rotation.z = data.pose.orientation.z
      t.transform.rotation.w = data.pose.orientation.w

      br.sendTransform(t)


    def odometryCallBack(self, data):

      if (self.alive % 10 == 0):
        pose = data.pose
        yaw_noise = np.random.normal(self.yaw_mu, self.yaw_sigma, size=1)
        x_noise = np.random.normal(self.x_y_mu, self.x_y_sigma, size=1)
        y_noise = np.random.normal(self.x_y_mu, self.x_y_sigma, size=1)
        pose.pose.position.x = pose.pose.position.x + x_noise
        pose.pose.position.y = pose.pose.position.y + y_noise

        q_orig = (
         pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w)

        q_rot = quaternion_from_euler(0, 0, yaw_noise)
        q_new = quaternion_multiply(q_orig, q_rot)
        pose.pose.orientation.x = q_new[0]
        pose.pose.orientation.y = q_new[1]
        pose.pose.orientation.z = q_new[2]
        pose.pose.orientation.w = q_new[3]
        test = PoseStamped()
        test.header = data.header
        test.pose = pose.pose
        self.pub_ndt_pose.publish(test)

        twist = TwistStamped()
        twist.twist = data.twist.twist
        twist.header = data.header
        self.pub_twist.publish(twist)

      self.alive = self.alive + 1


if __name__ == '__main__':
    try:
        rospy.init_node('gps_to_localization_pose')
        GPSToLocalizationPose()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])
