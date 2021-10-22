#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class SetInitialPose:

	def __init__(self):

		self.pub_gps_initial_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

		self.initial_pose = PoseWithCovarianceStamped()
		self.initial_pose.header.stamp = rospy.Time.now()
		self.initial_pose.header.frame_id = "map"

		self.initial_pose.pose.pose.position.x = rospy.get_param('~x', '0.0')
		self.initial_pose.pose.pose.position.y = rospy.get_param('~y', '0.0')
		self.initial_pose.pose.pose.position.z = rospy.get_param('~z', '0.0')
		self.initial_pose.pose.pose.orientation.x = rospy.get_param('~orientation_x', '0.0')
		self.initial_pose.pose.pose.orientation.y = rospy.get_param('~orientation_y', '0.0')
		self.initial_pose.pose.pose.orientation.z = rospy.get_param('~orientation_z', '0.0')
		self.initial_pose.pose.pose.orientation.w = rospy.get_param('~orientation_w', '1.0')

		self.initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]

		self.pub_gps_initial_pose.publish(self.initial_pose)


if __name__ == '__main__':
    try:

        rospy.init_node("set_initial_pose")
        SetInitialPose()
        rospy.spin()

    except rospy.ROSInterruptException:
        print "Unexpected error:", sys.exc_info()[0]
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])
