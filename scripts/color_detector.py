#!/usr/bin/env python
# Import our beloved friend. <3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from crazyflie_kinect_localization.msg import BallDetection
import imutils
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import time
import numpy as np
import math

class ColorDetectorNode(object):
    """ Detects Red Green Blue colored ping pong balls, from kinect hd RGB image topic, 
    publishes detection pixel coordinates accordingly."""

    def __init__(self):

        self.cf_color_lookup = rospy.get_param("cf_color_lookup", default={
                                               "GREEN": "cf1", "RED": "cf2", "BLUE": "cf3"})
        self.r_y_lower = rospy.get_param("~r_y_lower", default=150)
        self.r_y_upper = rospy.get_param("~r_y_upper", default=255)
        self.r_cr_lower = rospy.get_param("~r_cr_lower", default=140)
        self.r_cr_upper = rospy.get_param(
            "~r_cr_upper", default=250)  # was 110
        self.r_cb_lower = rospy.get_param("~r_cb_lower", default=50)
        self.r_cb_upper = rospy.get_param(
            "~r_cb_upper", default=120)  # was 150


        self.g_y_lower = rospy.get_param("~g_y_lower", default=0)
        self.g_y_upper = rospy.get_param("~g_y_upper", default=255)
        self.g_cr_lower = rospy.get_param("~g_cr_lower", default=70)
        self.g_cr_upper = rospy.get_param(
            "~g_cr_upper", default=105)  # was 110
        self.g_cb_lower = rospy.get_param("~g_cb_lower", default=120)
        self.g_cb_upper = rospy.get_param(
            "~g_cb_upper", default=140)  # was 150

        self.b_y_lower = rospy.get_param("~b_y_lower", default=0)
        self.b_y_upper = rospy.get_param("~b_y_upper", default=255)
        self.b_cr_lower = rospy.get_param("~b_cr_lower", default=110)
        self.b_cr_upper = rospy.get_param(
            "~b_cr_upper", default=140)  # was 110
        self.b_cb_lower = rospy.get_param("~b_cb_lower", default=140)
        self.b_cb_upper = rospy.get_param(
            "~b_cb_upper", default=255)  # was 150

        self.min_detection_radius = rospy.get_param(
            "~min_detection_radius", default=4)
        self.max_detection_radius = rospy.get_param(
            "~max_detection_radius", default=30)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        self.detection_image_pub = rospy.Publisher(
            "detection_image", Image, queue_size=2)
        self.depth_sub = rospy.Subscriber(
            "depth_image", Image, self.depth_callback)
        self.rate = rospy.Rate(30)
        self.pose_publishers = {
            "RED": rospy.Publisher("/{}/pose".format(self.cf_color_lookup["RED"]), PoseWithCovarianceStamped, queue_size=2),
            "GREEN": rospy.Publisher("/{}/pose".format(self.cf_color_lookup["GREEN"]), PoseWithCovarianceStamped, queue_size=2),
            "BLUE": rospy.Publisher("/{}/pose".format(self.cf_color_lookup["BLUE"]), PoseWithCovarianceStamped, queue_size=2),
        }

        # in calib_color.yaml file
        self.camera_matrix = [1.0815577461638704e+03, 0., 9.6726278213157309e+02, 0.,
                              1.0781758691523264e+03, 5.4445604167967088e+02, 0., 0., 1.]
        
        # self.camera_matrix = [1073.48539, 0., 970.95975, 0.,
        #                       1077.91738, 515.18869, 0., 0., 1.]

        self.camera_matrix = rospy.get_param(
            "~cameraMatrix", default={"data": [0 for i in range(9)]})["data"]

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

    def get_depth(self, x, y):
        if self.depth_frame is None:
            return 1000000000

        depth = self.depth_frame[y][x] / 1000.0
        return depth
        # x = (detection.x - self.c_x) * depth / self.f_x
        # y = (detection.y - self.c_y) * depth / self.f_y
        # cf_prefix = self.cf_color_lookup[key]

    def depth_callback(self, data):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        frame_gau_blur = cv2.GaussianBlur(frame, (3, 3), 0)
        # ycrcb = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2YCrCb)
        ycrcb = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
        # ycrcb = frame_gau_blur

        greenLower = (self.g_y_lower, self.g_cr_lower, self.g_cb_lower)
        greenUpper = (self.g_y_upper, self.g_cr_upper, self.g_cb_upper)

        redLower = (self.r_y_lower, self.r_cr_lower, self.r_cb_lower)
        redUpper = (self.r_y_upper, self.r_cr_upper, self.r_cb_upper)

        blueLower = (self.b_y_lower, self.b_cr_lower, self.b_cb_lower)
        blueUpper = (self.b_y_upper, self.b_cr_upper, self.b_cb_upper)

        green_mask = cv2.inRange(ycrcb, greenLower, greenUpper)
        red_mask = cv2.inRange(ycrcb, redLower, redUpper)
        blue_mask = cv2.inRange(ycrcb, blueLower, blueUpper)

        r_cnts = cv2.findContours(
            red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        r_cnts = imutils.grab_contours(r_cnts)
        if len(r_cnts) > 0 and False:
            ((x, y), radius, depth) = self.detect_contours(r_cnts)

            if radius is not None:
                cv2.circle(frame, (int(x), int(y)),
                           int(radius), (0, 0, 255), 3)
                cv2.circle(
                    frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["RED"], (int(
                    x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "RED"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                self.detections[msg.color] = (msg, depth)

        g_cnts = cv2.findContours(
            green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        g_cnts = imutils.grab_contours(g_cnts)
        if len(g_cnts) > 0:
            ((x, y), radius, depth) = self.detect_contours(g_cnts)
            if radius is not None:
                cv2.circle(frame, (int(x), int(y)),
                           int(radius), (0, 0, 255), 3)
                cv2.circle(
                    frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["GREEN"], (int(
                    x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()
                msg.color = "GREEN"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                self.detections[msg.color] = (msg, depth)

        b_cnts = cv2.findContours(
            blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        b_cnts = imutils.grab_contours(b_cnts)

        if len(b_cnts) > 0 and False:
            ((x, y), radius, depth) = self.detect_contours(b_cnts)
            if radius is not None:
                cv2.circle(frame, (int(x), int(y)),
                           int(radius), (0, 0, 255), 3)
                cv2.circle(
                    frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), int(2), (255, 0, 0), 2)
                cv2.putText(frame, self.cf_color_lookup["BLUE"], (int(
                    x) + 10, int(y)), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2)
                msg = BallDetection()

                msg.color = "BLUE"
                msg.frame_height = data.height
                msg.frame_width = data.width
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/kinect2"
                msg.x = x
                msg.y = y
                msg.radius = radius
                # self.detections[msg.color] = (msg, depth)


        

        # frame = cv2.bitwise_and(ycrcb, ycrcb, mask=green_mask)
        self.depth_clear_flag = True
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.detection_image_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_lower_half(self, data):
        data = data.flatten()
        mean = np.mean(data)
        if mean == 0.0 or len(data)==0:
            return [0]
        return [i for i in data if i <= mean]

    def detect_contours(self, contours):
        # print contours
        # contour_list = []
        # for contour in contours:
        #     approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
        #     area = cv2.contourArea(contour)
        #     if ((len(approx) > 8) & (len(approx) < 23) & (area > 250) ):
        #         print area
        #         contour_list.append(contour)
        # contours = contour_list
        print len(contours)
        
        possible_match = []
        radiuses = []
        for cont in contours:
            area = cv2.contourArea(cont)
            ((x, y), radius) = cv2.minEnclosingCircle(cont)

            # xlist = [x+i for i in range(-int(radius / 2), int(radius / 2))] 
            # ylist = [y+i for i in range(-int(radius / 2), int(radius / 2))] 
            # matrix = np.array(np.zeros(len(ylist)*len(xlist))).reshape((len(xlist), len(ylist)))
            # for i in range(len(xlist)):
            #     for j in range(len(ylist)):
            #         depth = self.get_depth(xlist[i], ylist[j])
            #         matrix[i][j] = depth
            # depth = np.mean(self.get_lower_half(matrix))
            depth = self.get_depth(x,y )


            if depth > 5.0 or depth < 0.1 or math.isnan(depth):
                continue
            if not (radius > self.min_detection_radius and radius < self.max_detection_radius):
                continue
            possible_match.append(((x, y), radius, depth))
            radiuses.append(radius)

        if len(possible_match) > 0:
            min_rad = min(radiuses)
            index = radiuses.index(min_rad)
            return possible_match[index]
        else:
            return ((None, None), None, None)


    def spin(self):
        while not rospy.is_shutdown():
            for key in self.keys:
                if self.detections.has_key(key):
                    detection, depth = self.detections[key]

                    x = (detection.x - self.c_x) * depth / self.f_x
                    y = (detection.y - self.c_y) * depth / self.f_y
                    cf_prefix = self.cf_color_lookup[key]

                    pose_msg = PoseWithCovarianceStamped()
                    pose_msg.header.frame_id = "/kinect2_rgb_optical_frame"
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.pose.pose.position.x = x
                    pose_msg.pose.pose.position.y = y
                    pose_msg.pose.pose.position.z = depth
                    pose_msg.pose.pose.orientation.w = 1.0

                    self.pose_publishers[key].publish(pose_msg)

                    br = tf.TransformBroadcaster()
                    br.sendTransform((x, y, depth),
                                    tf.transformations.quaternion_from_euler(
                                        0, 0, 0),
                                    rospy.Time.now(),
                                    cf_prefix,
                                    "/kinect2_rgb_optical_frame")
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kinect_color_detection_node")
    node = ColorDetectorNode()
    node.spin()
