#! /usr/bin/env python3

"""
Translates images from stonefish fls to norbit fls message
"""

import rospy, tf
from nortek_dvl333_driver.msg import BottomTrack
from cola2_msgs.msg import DVL
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
import numpy as np


class dvl_to_nortekDVL(object):
    def __init__(self):
        
        self.dvL_message = BottomTrack()

        self.dvl_pub = rospy.Publisher("/dvl/bottomtrack", BottomTrack, queue_size=1)
        self.sim_sub = rospy.Subscriber("/lolo/sim/dvl", DVL, self.cb, queue_size=1)

    
    def cb(self, msg: DVL):
        #receive image from similated FLS and publish it in norbit fls message format
        #print("DVL message recevied")
        altitude = msg.altitude


        self.dvL_message.distBeam0 = altitude
        self.dvL_message.distBeam1 = altitude
        self.dvL_message.distBeam2 = altitude
        self.dvL_message.distBeam3 = altitude

        #Not correct. But not just atm in lolo so whatever...
        self.dvL_message.velBeam0 = msg.velocity.x
        self.dvL_message.velBeam1 = msg.velocity.y
        self.dvL_message.velBeam2 = msg.velocity.z
        self.dvL_message.velBeam3 = msg.velocity.z 

        #TODO Publish nortek-like DVL message for Lolo
        self.dvl_pub.publish(self.dvL_message)
        


if __name__ == "__main__":
    rospy.init_node("sim_to_dvl")

    fls = dvl_to_nortekDVL()
    rospy.spin()



