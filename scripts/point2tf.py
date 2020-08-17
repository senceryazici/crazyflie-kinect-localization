#!/usr/bin/python
import rospy 
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped, TransformStamped

class TransformPublisherNode:
    def __init__(self):
        self.bcaster = tf.TransformBroadcaster()
        self.cf = rospy.get_param("~cf", default="cf1")
        rospy.Subscriber("/{}/pose".format(self.cf), PointStamped, self.callback)

    def callback(self, data):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = data.header.stamp
        tf_msg.header.frame_id = data.header.frame_id
        tf_msg.child_frame_id = "/{}".format(self.cf)
        tf_msg.transform.translation = data.point
        tf_msg.transform.rotation.w = 1.0

        self.bcaster.sendTransformMessage(tf_msg)
        
if __name__ == "__main__":
    rospy.init_node("tf_pub_node")
    node = TransformPublisherNode()
    rospy.spin()