#!/usr/bin/env python3

import rospy

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry




class OdomPublisher:

    def __init__(self):

        topic_pose = rospy.get_param('~topic_pose', '/pose')
        topic_odom = rospy.get_param('~topic_odom', '/odom')

        self.pub_odometry = rospy.Publisher(topic_odom, Odometry, queue_size=1)

        self.sub_pose = rospy.Subscriber(topic_pose, PoseStamped, self.poseCallBack)


    def poseCallBack(self, data):
        odom = Odometry()
        odom.header = data.header
        odom.pose.pose = data.pose
        self.pub_odometry.publish(odom)


if __name__ == '__main__':
    try:
        rospy.init_node("odom_publisher")
        OdomPublisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Unexpected error:", sys.exc_info()[0])
        rospy.logfatal("Unexpected error:", sys.exc_info()[0])


    
    
