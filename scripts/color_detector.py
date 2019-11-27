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
        # get parameters
        self.cf_color_lookup = rospy.get_param("cf_color_lookup", default={"GREEN": "cf2", "red": "cf1", "blue": "cf3"})
        self.y_lower = rospy.get_param("~y_lower", default=0)
        self.y_upper = rospy.get_param("~y_upper", default=255)
        self.cr_lower = rospy.get_param("~cr_lower", default=70)
        self.cr_upper = rospy.get_param("~cr_upper", default=105) # was 110
        self.cb_lower = rospy.get_param("~cb_lower", default=120)
        self.cb_upper = rospy.get_param("~cb_upper", default=140) # was 150
        self.min_detection_radius = rospy.get_param("~min_detection_radius", default=9.5)


        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/detection", BallDetection, queue_size=10) 
        self.detection_image_pub = rospy.Publisher("/detection_image", Image, queue_size=2)


    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
        ycrcb = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2YCrCb)

        greenLower = (self.y_lower, self.cr_lower, self.cb_lower)
        greenUpper = (self.y_upper, self.cr_upper, self.cb_upper)

        green_mask = cv2.inRange(ycrcb, greenLower, greenUpper)
        # p_b[i / 2] = (cr > 110 && cr < 140 && cb > 140 && cb < 255) ? max : min;//cr 100-130 BLUE
        # p[i / 2] = (cr > 140 && cr < 190 && cb > 70  && cb < 120) ? max : min; RED

        cnts = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > self.min_detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 3)
                cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["GREEN"], (int(x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "GREEN"
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
