#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse

import rospy
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry  # Import Odometry message

odom = Odometry()
odom_ = Odometry()
odom_prv = Odometry()

def nav_odom_callback(data):
    global odom_
    odom_ = data

if __name__ == '__main__':
    rospy.init_node('publish_initial_pose')
    odom_sub = rospy.Subscriber('liorf/mapping/odometry', Odometry, nav_odom_callback, queue_size=30)
    pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    quat = tf.transformations.quaternion_from_euler(args.roll, args.pitch, args.yaw)
    xyz = [args.x, args.y, args.z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
    initial_pose.header.stamp = rospy.Time().now()
    initial_pose.header.frame_id = 'map'
    rospy.sleep(1)
    rospy.loginfo('Last Pose: {} {} {} {} {} {}'.format(
        args.x, args.y, args.z, args.yaw, args.pitch, args.roll, ))
    pub_pose.publish(initial_pose)
