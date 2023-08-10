#! /usr/bin/env python3

"""
We want to simulate the INS driver in the simulation.
Stonefish has no INS by itself, so we'll fake it by fusing GPS, IMU etc into the same
message the real INS driver would produce.
Any dead-reckoning package should read this INS message and publish lolo/base_link -> map connections in TF
"""

import rospy, tf
from ixblue_ins_msgs.msg import Ins
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped, Point
from smarc_msgs.srv import UTMToLatLon
from nav_msgs.msg import Odometry

from geographic_msgs.msg import GeoPoint
from geodesy.utm import *
from geodesy import *

from sensor_msgs.msg import Imu

# Ins message:
# int8 ALT_REF_GEOID=0
# int8 ALT_REF_ELLIPSOID=1
# std_msgs/Header header
  # uint32 seq
  # time stamp
  # string frame_id
# float64 latitude
# float64 longitude
# int8 altitude_ref
# float32 altitude
# float64[9] position_covariance
# float32 heading
# float32 roll
# float32 pitch
# float64[9] attitude_covariance
# geometry_msgs/Vector3 speed_vessel_frame
  # float64 x
  # float64 y
  # float64 z
# float64[9] speed_vessel_frame_covariance


class INS(object):
    def __init__(self,
                 robot_name="lolo"):

        self.base_link = robot_name+"/base_link"
        odom_topic = "/"+robot_name+"/sim/odom"
        imu_topic =  "/"+robot_name+"/sim/imu"
        
        self.origin_lat = rospy.get_param("~origin_latitude", 6) #origin latitude in degrees
        self.origin_lon = rospy.get_param("~origin_longitude", 7) #origin longitude in degrees

        print("sim to INS origin lat: " + str(self.origin_lat))
        print("sim to INS origin lon: " + str(self.origin_lon))


        self.ins_msg = Ins()
        self.ins_msg.header.frame_id = self.base_link
        self.ins_msg.altitude_ref = Ins.ALT_REF_GEOID

        self.ins_pub = rospy.Publisher("/"+robot_name+"/core/ins", Ins, queue_size=1)

        #convert origin to UTM
        self.origin_wgs84 = GeoPoint()
        self.origin_wgs84.latitude = self.origin_lat
        self.origin_wgs84.longitude = self.origin_lon
        self.origin_wgs84.altitude = 0
        self.origin_utm = fromMsg(self.origin_wgs84)

        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=1)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.base_link
        self.imu_pub = rospy.Publisher("/"+robot_name+"/core/imu", Imu, queue_size=1)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_cb, queue_size=1)

    
    def imu_cb(self, msg):
        #Republish imu message with inverted axes
        self.imu_msg.angular_velocity.x = msg.angular_velocity.x
        self.imu_msg.angular_velocity.y = -msg.angular_velocity.y
        self.imu_msg.angular_velocity.z = -msg.angular_velocity.z

        self.imu_msg.linear_acceleration.x = msg.linear_acceleration.x
        self.imu_msg.linear_acceleration.y = -msg.linear_acceleration.y
        self.imu_msg.linear_acceleration.z = -msg.linear_acceleration.z

        self.imu_pub.publish(self.imu_msg)


    def odom_cb(self, msg):
        """
        update INS message from odometry
        """
        # Add odom positon to UTM position
        utmpos = UTMPoint()
        utmpos.altitude = self.origin_utm.altitude
        utmpos.band = self.origin_utm.band
        utmpos.zone = self.origin_utm.zone
        utmpos.northing = self.origin_utm.northing
        utmpos.easting = self.origin_utm.easting

        #utmpos.easting += msg.pose.pose.position.x
        #utmpos.northing += msg.pose.pose.position.y

        utmpos.northing += msg.pose.pose.position.x
        utmpos.easting += msg.pose.pose.position.y

        latlon = utmpos.toMsg()

        self.ins_msg.latitude = latlon.latitude
        self.ins_msg.longitude = latlon.longitude

        # turn that quat into rpy
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        
        self.ins_msg.roll = 180 + 180.0/3.1415*rpy[0]
        if self.ins_msg.roll > 180: self.ins_msg.roll-=360 #range : -180 180
        self.ins_msg.pitch = -(180.0/3.1415*rpy[1])
        self.ins_msg.heading = 180.0/3.1415*rpy[2]

        # this altitude is altitude in the global sense, from sea level!
        self.ins_msg.altitude = -msg.pose.pose.position.z
        self.ins_msg.speed_vessel_frame.x = msg.twist.twist.linear.x
        self.ins_msg.speed_vessel_frame.y = -msg.twist.twist.linear.y
        self.ins_msg.speed_vessel_frame.z = -msg.twist.twist.linear.z


    def publish(self):
        self.ins_msg.header.seq += 1
        self.ins_msg.header.stamp = rospy.Time.now()
        self.ins_pub.publish(self.ins_msg)



if __name__ == "__main__":
    rospy.init_node("sim_to_ins")

    ins = INS()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        ins.publish()
        rate.sleep()




