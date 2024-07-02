#! /usr/bin/env python3

"""
Translates images from stonefish fls to norbit fls message
"""

import rospy, tf
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros_numpy
import math


class LC_to_PC2(object):
    def __init__(self):
    

        self.pc2_pub = rospy.Publisher("/lolo/sensors/mbes/pc2_bathymetry", PointCloud2, queue_size=1)
        #self.lc_pub = rospy.Publisher("/lolo/sensors/mbes/laserscan_bathymetry", LaserScan, queue_size=1)
        self.sim_sub = rospy.Subscriber("/lolo/sim/MBES", LaserScan, self.cb, queue_size=1)

    
    def cb(self, msg: LaserScan):
        print("Converting Laserscan msg to pointcloud2 msg.")

        angles = []
        ranges = []
        intensities = []

        for i in range(len(msg.ranges)):
            angles.append(-1.0* (msg.angle_min + i*msg.angle_increment) )
            r = msg.ranges[i] if msg.ranges[i] < msg.range_max-2 else math.nan
            ranges.append(r)
            intensities.append(1)
                

        angles = np.asarray(angles)
        ranges = np.asarray(ranges)
        intensities = np.asarray(intensities)

        # Build the recarray with points and intensities
        pointcloud = np.recarray((1,len(ranges)),dtype=[('x', np.float32),
                                                        ('y', np.float32),
                                                        ('z', np.float32),
                                                        ('intensity', np.uint8)])
        pointcloud['z'] = np.zeros(len(ranges))
        pointcloud['y'] = ranges * np.sin(angles)
        pointcloud['x'] = ranges * np.cos(angles)
        pointcloud['intensity'] = intensities

        # Do some magic
        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(pointcloud)
        pointcloud_msg.header.stamp = rospy.Time.now()
        pointcloud_msg.header.frame_id = 'lolo/mbes_link'

        self.pc2_pub.publish(pointcloud_msg)
        
        


if __name__ == "__main__":
    rospy.init_node("sim_to_mbes_pc2")

    fls = LC_to_PC2()
    rospy.spin()



