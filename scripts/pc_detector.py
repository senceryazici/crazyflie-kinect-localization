#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy
import tf

import struct

import pcl_ros

from os.path import expanduser
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
import ctypes
from sensor_msgs.msg import Image, PointCloud2


class Detector:

    def __init__(self):
        #  get label map and inference graph from params
        self.pc = None
        self.sub = rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.pc_callback)

    def pc_callback(self, data):
        self.pc = data
        

    def spin(self):
        while not rospy.is_shutdown():
            if self.pc is None:
                continue
            gen = pc2.read_points(self.pc, skip_nans=True)
            int_data = list(gen)
            for x in int_data:
                test = x[3] 
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f' ,test)
                i = struct.unpack('>l',s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x0000FF00)
                print r,g,b # prints r,g,b values in the 0-255 range
                            # x,y,z can be retrieved from the x[0],x[1],x[2]

if __name__ == '__main__':
    rospy.init_node('dodo_detector_ros', log_level=rospy.INFO)

    try:
        node = Detector()
        node.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')