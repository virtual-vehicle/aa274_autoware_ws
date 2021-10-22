#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import sys
import os
import signal
import subprocess

class RVIZGoalTransform:

    def __init__(self):
      self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
      self.sub_goal = rospy.Subscriber("/move_base_simple/goal_map", PoseStamped, self.goalCallBack)

    def goalCallBack(self, data):
       data.header.frame_id = "world"
       self.pub_goal.publish(data)


if __name__ == '__main__':
    try:
        rospy.init_node('goal_transform')
        RVIZGoalTransform()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Unexpected error:", sys.exc_info()[0])
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])

