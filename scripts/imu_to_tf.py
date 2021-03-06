#!/usr/bin/env python  
import roslib
#roslib.load_manifest('imu_to_tf')
import rospy

import sensor_msgs
import tf


br = None
child_frame_tf 		= "base_link"
parent_frame_tf 	= "base_stabilized"

def imu_msg_callback(imu_msg):

    #euler = tf.transformations.euler_from_quaternion(imu_msg.orientation)
    #roll = euler[0]
    #pitch = -euler[1]
    #yaw = -euler[2]
    
    #quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                     
    br.sendTransform((0, 0, 0),
                    (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w),
                     rospy.Time.now(),
                     child_frame_tf,
                     parent_frame_tf)

if __name__ == '__main__':
    rospy.init_node('imu_to_tf')
    
    br = tf.TransformBroadcaster()
    
    rospy.Subscriber('/imu/data',
                     sensor_msgs.msg.Imu,
                     imu_msg_callback)
    rospy.spin()