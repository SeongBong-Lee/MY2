#!/usr/bin/env python3

# python glob module
import numpy as np
import math
import time
import struct
# ROS module
import rospy

from sensor_msgs.msg import Imu  # temporarily deprecated
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped  # temporarily deprecated
from tf2_msgs.msg import TFMessage  # temporarily deprecated
from interfaces_msgs.msg import AutoNavigation, NavigationDetail
import tf
from tf.transformations import *

# Function to convert quaternion to Euler angles
def quaternion_to_euler_angle(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# UDP socket
import json
import socket

# params
#HOST = "172.21.1.12"  # Host, Robot IP
HOST = 'localhost' # Host, Robot IP
PORT = 12301  # define port here
SIZE = 1024
addr = ''
CNT = 0

# Global pose
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.0
# UTM Nav Format
POS_N = 0.0
POS_E = 0.0
POS_DWN = 0.0
OR_YAW = 0
OR_PITCH = 0
OR_ROLL = 0

QUAT_X = 0
QUAT_Y = 0
QUAT_Z = 0
QUAT_W = 0
QUAT = [0, 0, 0, 0]

# struct parameter
h1 = np.uint32(1)
h2 = np.uint32(1)
h3 = np.uint32(1)
h4 = np.uint32(16)
h5 = np.uint32(16)

error_flag = False

utm_zone = 52
utm_letter = 0
utm_x = 0
utm_y = 0

odom_ = Odometry()


def nav_odom_callback(data):
    global odom_
    odom_ = data


rospy.init_node('utm_odom_udp', anonymous=False)
sub_odom = rospy.Subscriber('utm_odom', Odometry, nav_odom_callback, queue_size=120)
rate = rospy.Rate(100)

if __name__ == '__main__':
    client_set = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_set.connect((HOST, PORT))

    while not rospy.is_shutdown():
        POS_N = odom_.pose.pose.position.x  # lat
        POS_E = odom_.pose.pose.position.y  # lon
        POS_DWN = odom_.pose.pose.position.z  # elevation

        QUAT[0] = odom_.pose.pose.orientation.x
        QUAT[1] = odom_.pose.pose.orientation.y
        QUAT[2] = odom_.pose.pose.orientation.z
        QUAT[3] = odom_.pose.pose.orientation.w

        OR_ROLL, OR_PITCH, OR_YAW = quaternion_to_euler_angle(QUAT[3], QUAT[2], QUAT[1], QUAT[0])  # Quat to Euler Conversion

        # 위치정보(North, East, Down), 자세정보(Heading), SLAM초기 좌표(전역과 동기화된 좌표)
        LocationDataStruct = struct.pack('IIIII?ffffBBff',
                                          h1,  # uint32 1
                                          h2,  # uint32 1
                                          h3,  # uint32 1
                                          h4,  # uint32 16
                                          h5,  # uint32 16
                                          error_flag,  # bool, 1 : error & else : normal
                                          POS_N,  # north
                                          POS_E,  # east
                                          POS_DWN,  # down
                                          OR_YAW,  # heading
                                          utm_zone,  # utm zone
                                          utm_letter,  # utm letter
                                          utm_x,  # SLAM init UTM zone
                                          utm_y)  # SLAM init UTM Zone

        client_set.sendto(LocationDataStruct, (HOST, PORT))
        rate.sleep()