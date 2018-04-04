#!/usr/bin/env python
import roslib
# roslib.load_manifest('pose_to_odom')
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

br = None
parent_frame = "odom"
child_frame = "base_footprint"


def pose_msg_callback(pose_msg):

    br.sendTransform(pose_msg.pose.pose.position,
                     pose_msg.pose.pose.orientation,
                     rospy.Time.now(),
                     child_frame,
                     parent_frame)


if __name__ == '__main__':
    rospy.init_node('pose_to_odom')

    br = tf.TransformBroadcaster()

    rospy.Subscriber('/scan_pose',
                     PoseWithCovarianceStamped,
                     pose_msg_callback)

    rospy.spin()