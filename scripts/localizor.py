#!/usr/bin/env python
# Import our beloved friend. <3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from crazyflie_kinect_localization.msg import BallDetection
import imutils

class LocalizerNode(object):
    """ -- """

    def __init__(self):
        pass
        self.detection_sub = rospy.Subscriber("/detection", BallDetection, self.detection_callback)
        # self.kinect_depth_sub = rospy.Subscriber()


    def detection_callback(self, data):
        pass



if __name__ == "__main__":
    rospy.init_node("kinect_color_detection_node")
    node = LocalizerNode()
    rospy.spin()
