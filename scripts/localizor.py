#!/usr/bin/env python
# Import our beloved friend. <3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from crazyflie_kinect_localization.msg import BallDetection
from geometry_msgs.msg import PointStamped
import imutils
import math
import numpy as np
import tf

class LocalizerNode(object):
    """ -- """

    def __init__(self):
        self.cf_color_lookup = rospy.get_param("cf_color_lookup", default={"GREEN": "cf2", "RED": "cf1", "BLUE": "cf3"})

        self.detection_sub = rospy.Subscriber("detection", BallDetection, self.detection_callback)
        self.depth_sub = rospy.Subscriber("depth_image", Image, self.depth_callback)

        # in calib_color.yaml file
        self.camera_matrix = [ 1.0815577461638704e+03, 0., 9.6726278213157309e+02, 0.,
        1.0781758691523264e+03, 5.4445604167967088e+02, 0., 0., 1. ]

        self.camera_matrix = rospy.get_param("~cameraMatrix", default={"data": [0 for i in range(9)]})["data"]

        # camera matrix (intrinsic matrix)
        # fx  0  cx
        # 0  fy  cy
        # 0  0  1
        self.f_x = self.camera_matrix[0]
        self.c_x = self.camera_matrix[2]
        self.f_y = self.camera_matrix[4]
        self.c_y = self.camera_matrix[5]

        self.bridge = CvBridge()
        self.depth_frame = None
        self.keys = ["GREEN", "RED", "BLUE"]

        self.detections = {
        }
        self.positions = {

        }

    def depth_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        for key in self.keys:
            if self.detections.has_key(key):
                detection = self.detections[key]

                depth = self.depth_frame[detection.y][detection.x] / 1000.0
                x = (detection.x - self.c_x) * depth / self.f_x
                y = (detection.y - self.c_y) * depth / self.f_y
                cf_prefix = self.cf_color_lookup[key]

                br = tf.TransformBroadcaster()
                br.sendTransform((x, y, depth),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),
                                cf_prefix,
                                "/kinect2_rgb_optical_frame")

    def detection_callback(self, data):
        self.detections[data.color] = data


if __name__ == "__main__":
    rospy.init_node("kinect_depth_detection_node")
    node = LocalizerNode()
    rospy.spin()
