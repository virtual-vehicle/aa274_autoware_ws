#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import *
from std_msgs.msg import Float32



import sys
import os
import signal
import subprocess
import numpy as np
import math

def calculateDistance(pose1,pose2):  
     dist = math.sqrt((pose2.position.x - pose1.position.x)**2 + (pose2.position.y - pose1.position.y)**2)  
     return dist  



class EKFEvaluation:

    def __init__(self):
      self.sub_gps_pose = rospy.Subscriber('/gps_local/pose', PoseStamped, self.gpsPoseCallBack)
      self.sub_localization_pose = rospy.Subscriber('/localization_pose', PoseStamped, self.localizationPoseCallBack)
      self.sub_ekf_pose = rospy.Subscriber('/ekf_pose_with_covariance', PoseWithCovarianceStamped, self.ekfPoseCallBack)

      self.pub_dist_gps_localization = rospy.Publisher('/evaluation/dist_gps_localization', Float32, queue_size=10)
      self.pub_dist_gps_ekf = rospy.Publisher('/evaluation/dist_gps_ekf', Float32, queue_size=10)
      self.pub_cov_x_ekf = rospy.Publisher('/evaluation/covariance_x_ekf', Float32, queue_size=10)

      self.gps_pose = PoseStamped()
      self.localization_pose = PoseStamped()
      self.ekf_pose_with_covariance = PoseWithCovarianceStamped()

      self.dist_gps_ekf_arr = []
      self.dist_gps_localization_arr = []

    def gpsPoseCallBack(self, data):
      self.gps_pose = data  

    def localizationPoseCallBack(self, data):
      self.localization_pose = data  

      # publish distance error between gps_pose and localization_pose 
      value = Float32()
      value.data = calculateDistance(self.gps_pose.pose, self.localization_pose.pose)
      self.pub_dist_gps_localization.publish(value)

      self.dist_gps_localization_arr.append(value.data)


    def ekfPoseCallBack(self, data):
      self.ekf_pose_with_covariance = data

      # publish distance error between gps_pose and ekf_pose 
      value = Float32()
      value.data = calculateDistance(self.gps_pose.pose, self.ekf_pose_with_covariance.pose.pose)
      self.pub_dist_gps_ekf.publish(value)

      self.dist_gps_ekf_arr.append(value.data)

      if (len(self.dist_gps_ekf_arr) > 0 and len(self.dist_gps_localization_arr) > 0):
          mean_dist_ekf = sum(self.dist_gps_ekf_arr) / float(len(self.dist_gps_ekf_arr)) 
          mean_dist_localization = sum(self.dist_gps_localization_arr) / float(len(self.dist_gps_localization_arr)) 
      else:
          mean_dist_ekf = 0.0
          mean_dist_localization = 0.0

      rospy.loginfo("mean_dist_localization: %f mean_dist_ekf: %f", mean_dist_localization, mean_dist_ekf)

      # publish covariance x
      value = Float32()
      value.data = self.ekf_pose_with_covariance.pose.covariance[0]    
      self.pub_cov_x_ekf.publish(value)




if __name__ == '__main__':
    try:
        rospy.init_node('localization_evaluation') # Initialize ROS
        EKFEvaluation()
        rospy.spin()

    except rospy.ROSInterruptException:
        print "Unexpected error:", sys.exc_info()[0]
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])

