#!/usr/bin/env python
# Import our beloved friend. <3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from crazyflie_kinect_localization.msg import BallDetection
import imutils

class ColorDetectorNode(object):
    """ Detects Red Green Blue colored ping pong balls, from kinect hd RGB image topic, 
    publishes detection pixel coordinates accordingly."""

    def __init__(self):
        
        self.cf_color_lookup = rospy.get_param("cf_color_lookup", default={"GREEN": "cf2", "RED": "cf1", "BLUE": "cf3"})
        self.r_y_lower = rospy.get_param("~r_y_lower", default=150)
        self.r_y_upper = rospy.get_param("~r_y_upper", default=255)
        self.r_cr_lower = rospy.get_param("~r_cr_lower", default=140)
        self.r_cr_upper = rospy.get_param("~r_cr_upper", default=250) # was 110
        self.r_cb_lower = rospy.get_param("~r_cb_lower", default=50)
        self.r_cb_upper = rospy.get_param("~r_cb_upper", default=120) # was 150

        self.g_y_lower = rospy.get_param("~g_y_lower", default=0)
        self.g_y_upper = rospy.get_param("~g_y_upper", default=255)
        self.g_cr_lower = rospy.get_param("~g_cr_lower", default=70)
        self.g_cr_upper = rospy.get_param("~g_cr_upper", default=105) # was 110
        self.g_cb_lower = rospy.get_param("~g_cb_lower", default=120)
        self.g_cb_upper = rospy.get_param("~g_cb_upper", default=140) # was 150

        self.b_y_lower = rospy.get_param("~b_y_lower", default=0)
        self.b_y_upper = rospy.get_param("~b_y_upper", default=255)
        self.b_cr_lower = rospy.get_param("~b_cr_lower", default=110)
        self.b_cr_upper = rospy.get_param("~b_cr_upper", default=140) # was 110
        self.b_cb_lower = rospy.get_param("~b_cb_lower", default=140)
        self.b_cb_upper = rospy.get_param("~b_cb_upper", default=255) # was 150

        self.min_detection_radius = rospy.get_param("~min_detection_radius", default=4)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        self.detection_pub = rospy.Publisher("detection", BallDetection, queue_size=10) 
        self.detection_image_pub = rospy.Publisher("detection_image", Image, queue_size=2)


    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
        ycrcb = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2YCrCb)

        greenLower = (self.g_y_lower, self.g_cr_lower, self.g_cb_lower)
        greenUpper = (self.g_y_upper, self.g_cr_upper, self.g_cb_upper)

        redLower = (self.r_y_lower, self.r_cr_lower, self.r_cb_lower)
        redUpper = (self.r_y_upper, self.r_cr_upper, self.r_cb_upper)

        blueLower = (self.b_y_lower, self.b_cr_lower, self.b_cb_lower)
        blueUpper = (self.b_y_upper, self.b_cr_upper, self.b_cb_upper)

        green_mask = cv2.inRange(ycrcb, greenLower, greenUpper)
        red_mask = cv2.inRange(ycrcb, redLower, redUpper)
        blue_mask = cv2.inRange(ycrcb, blueLower, blueUpper)

        r_cnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        r_cnts = imutils.grab_contours(r_cnts)
        if len(r_cnts) > 0:
            c = max(r_cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > self.min_detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 3)
                cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["RED"], (int(x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "RED"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                self.detection_pub.publish(msg)

        g_cnts = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        g_cnts = imutils.grab_contours(g_cnts)
        if len(g_cnts) > 0:
            c = max(g_cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > self.min_detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 3)
                cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["GREEN"], (int(x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "GREEN"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                self.detection_pub.publish(msg)

        b_cnts = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        b_cnts = imutils.grab_contours(b_cnts)
        if len(b_cnts) > 0:
            c = max(b_cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > self.min_detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 3)
                cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["BLUE"], (int(x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "BLUE"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                self.detection_pub.publish(msg)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.detection_image_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(e)
        


if __name__ == "__main__":
    rospy.init_node("kinect_color_detection_node")
    node = ColorDetectorNode()
    rospy.spin()
