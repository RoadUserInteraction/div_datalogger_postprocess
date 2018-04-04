#!/usr/bin/env python  
import roslib
#roslib.load_manifest('imu_to_tf')
import rospy
import math
import sensor_msgs
import tf
from tf.transformations import quaternion_multiply

br = None
parent_frame_tf0 = "base_footprint"
parent_frame_tf1 = "handlebar_link"
rot_quat = (math.sqrt(2)/2, math.sqrt(2)/2, 0, 0)
def imu_msg_callback(imu_msg):
    '''if imu_msg.header.frame_id == 'imu0_link':
        parent_frame_tf = parent_frame_tf0
    else:
        parent_frame_tf = parent_frame_tf1'''

    imu_orientation = quaternion_multiply(rot_quat, (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w))
    br.sendTransform((0, 0, 0),
                     imu_orientation,
                     rospy.Time.now(),
                     imu_msg.header.frame_id,
                     parent_frame_tf0)


if __name__ == '__main__':
    rospy.init_node('imu_to_tf')
    
    br = tf.TransformBroadcaster()
    
    rospy.Subscriber('/imu0/data',
                     sensor_msgs.msg.Imu,
                     imu_msg_callback)

    rospy.spin()