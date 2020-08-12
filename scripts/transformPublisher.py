import rospy 
import tf


class TransformPublisherNode:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(30)
        self.source_frame = "world"
        self.target_frame = "cf1"

    def spin(self):
        while not rospy.is_shutdown():
            self.listener.waitForTransform(self.source_frame, self.target_frame, rospy.Time(), rospy.Duration(4.0))
            t = self.listener.getLatestCommonTime(self.source_frame, self.target_frame)
            transform = self.listener.lookupTransform(self.source_frame, self.target_frame, t)
            print transform
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("tf_pub_node")
    node = TransformPublisherNode()
    node.spin()