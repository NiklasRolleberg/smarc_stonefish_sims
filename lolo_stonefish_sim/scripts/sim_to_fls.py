#! /usr/bin/env python3

"""
Translates images from stonefish fls to norbit fls message
"""

import rospy, tf
from norbit_wbms_driver.msg import WaterColumn
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
import numpy as np


class imag_to_fls(object):
    def __init__(self):
        self.fls_message = WaterColumn()
        self.fls_message.preamble = 3735928559
        self.fls_message.type = 2
        self.fls_message.size = 1050816
        self.fls_message.snd_velocity = 1500.0
        self.fls_message.sample_rate = 4882.8125
        self.fls_message.num_beams = 512
        self.fls_message.num_samples = 1024
        self.fls_message.Time = 14927.26219561091
        self.fls_message.dtype = 3
        self.fls_message.t0 = -1
        self.fls_message.gain = 19.132028579711914
        self.fls_message.swath_dir = 0.0
        self.fls_message.swath_open = 2.6179938316345215
        self.fls_message.tx_freq = 400000.0
        self.fls_message.tx_bw = 80000.0
        self.fls_message.tx_len = 0.0005000000237487257
        self.fls_message.tx_amp = 15
        self.fls_message.ping_rate = 3.5594351291656494
        self.fls_message.ping_number = 165957
        self.fls_message.time_net = 14927.82797181103
        self.fls_message.beams = 512
        self.fls_message.vga_t1 = 0
        self.fls_message.vga_g1 = 0.0
        self.fls_message.vga_t2 = 256
        self.fls_message.vga_g2 = 39.32160186767578
        self.fls_message.tx_angle = 0.0
        self.fls_message.tx_voltage = 0.0
        self.fls_message.beam_dist_mode = 2
        self.fls_message.sonar_mode = 0
        self.fls_message.gate_tilt = 0.0

        self.fls_pub = rospy.Publisher("/lolo/sensors/fls/wc/data", WaterColumn, queue_size=1)
        self.image_sub = rospy.Subscriber("/lolo/sim/FLS/image", Image, self.image_cb, queue_size=1)

    
    def image_cb(self, msg):
        #receive image from similated FLS and publish it in norbit fls message format
        #print("fls image recevied")

        # Get the pixel and beam directions arrays.
        pixel_data = np.array([float(d) for d in msg.data])
        pixel_data = pixel_data[::-1]
        #pixel_data = np.reshape(pixel_data, (self.fls_message.num_samples, self.fls_message.num_beams))
        #pdb.set_trace()
        directions = np.linspace(-self.fls_message.swath_open, self.fls_message.swath_open, num=self.fls_message.num_beams)
        
        # Build the image.
        image = Image()
        image.header.stamp = rospy.Time.now()
        image.height = self.fls_message.num_samples
        image.width = self.fls_message.num_beams
        image.encoding = 'mono8'  # TODO This should dynamically change
                                   # depending on the data's dtype...
        # Build the multi array.
        array = Float32MultiArray()
        img_dim = MultiArrayDimension()
        img_dim.label = "pixel_data"
        img_dim.size = image.height * image.width
        img_dim.stride = img_dim.size * 4
        dir_dim = MultiArrayDimension()
        dir_dim.label = "directions"
        dir_dim.size = len(directions)
        dir_dim.stride = dir_dim.size * 4
        array.layout.dim = [img_dim, dir_dim]
        array.data = np.array(pixel_data).astype(float).tolist() + directions.tolist()

        self.fls_message.watercolumn_raw = array

        self.fls_pub.publish(self.fls_message)
        


if __name__ == "__main__":
    rospy.init_node("sim_to_fls")

    fls = imag_to_fls()
    rospy.spin()
    #rate = rospy.Rate(5)
    #while not rospy.is_shutdown():
    #    ins.publish()
    #    rate.sleep()




