#!/usr/bin/env python3

#python glob module
import numpy as np
import math
import pyproj
from pyproj import transform
#ROS module
import rospy
from sensor_msgs.msg import Imu #temporarily deprecated
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped #temporarily deprecated
from tf2_msgs.msg import TFMessage #temporarily deprecated
from interfaces_msgs.msg import AutoNavigation, NavigationDetail
import tf
from tf.transformations import *

#set variables
pose = PoseStamped()
utm_data = NavigationDetail()
utm_data_prv = NavigationDetail()
odom = Odometry()
odom_ = Odometry()
odom_prv = Odometry()
counter = 0
x = 0.
y = 0.
mil2deg = 0.0573;
deg2rad = np.pi/180;

def deg_to_rad(deg):
    return deg * (np.pi / 180)

def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

def nav_callback(data):
  global counter
  utm_data.east = data.nav_detail.east
  utm_data.north = data.nav_detail.north
  utm_data.elevation = data.nav_detail.elevation
  utm_data.heading = data.nav_detail.heading + 1300
  utm_data.pitch = data.nav_detail.pitch
  utm_data.roll = data.nav_detail.roll
  counter += 1

def nav_odom_callback(data):
  global odom_
  odom_ = data

def utm_to_latlon(easting, northing, northernHemisphere=True):
        if not northernHemisphere:
            northing = -northing

        utm_proj = pyproj.Proj(
            proj='utm',
            zone=52,
            ellps='WGS84',
            datum='WGS84',
            south=not northernHemisphere)

        latlon_proj = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        lon, lat = pyproj.transform(utm_proj, latlon_proj, easting, northing)

        return lat, lon


def lonlat_to_xy(Lon, Lat):
  P = pyproj.Proj(proj='utm', zone=31, ellps='WGS84', preserve_units=True)
  x,y = P(Lon, Lat)   
  return x,y

def quaternion_from_euler_(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q
#==================================================#
#ROS node init
rospy.init_node('nav_to_odom')
odom_sub_1 = rospy.Subscriber('liorf/mapping/odometry', Odometry, nav_odom_callback, queue_size=30)
odom_pub_0 = rospy.Publisher('/utm_odom', Odometry, queue_size=15)
rate = rospy.Rate(100.0) #pub rate, follow realtime
#====================================================#

while not rospy.is_shutdown():
  odom.header.frame_id = 'map'
  odom.child_frame_id = 'odom'
  odom.header.stamp = rospy.Time.now()
  
  conv_x = odom_.pose.pose.position.x + 4155175.36 #map origin(0,0,0) offset
  conv_y = odom_.pose.pose.position.y + 321259.48 #map origin(0,0,0) offset
  # lat, lon = utm_to_latlon(utm_data.east, utm_data.north)
  lat, lon = utm_to_latlon(conv_x, conv_y)
  #publish to odom
  odom.pose.pose.position.x = lat
  odom.pose.pose.position.y = lon
  odom.pose.pose.position.z = odom_.pose.pose.position.z + 38
  
  odom.pose.pose.orientation.w = odom_.pose.pose.orientation.w
  odom.pose.pose.orientation.x = odom_.pose.pose.orientation.x
  odom.pose.pose.orientation.y = odom_.pose.pose.orientation.y
  odom.pose.pose.orientation.z = odom_.pose.pose.orientation.z

  odom_pub_0.publish(odom)

  rate.sleep()