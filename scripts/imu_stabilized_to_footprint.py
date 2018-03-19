#!/usr/bin/env python  
import roslib
import rospy

import sensor_msgs
import tf


br = None
child_frame_tf 		= "base_stabilized_corrected"
parent_frame_tf 	= "base_footprint"

def imu_msg_callback(imu_msg):

    euler = tf.transformations.euler_from_quaternion([imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = -euler[2]
    #print yaw
    orientation_new = tf.transformations.quaternion_from_euler(0,0,yaw)
    #quaternion = tf.transformations.quaternion_from_euler([roll, pitch, yaw])
    #print orientation_new                   
    br.sendTransform((0, 0, 0),
                    (orientation_new[0], orientation_new[1], orientation_new[2], orientation_new[3]),
                     rospy.Time.now(),
                     child_frame_tf,
                     parent_frame_tf)

if __name__ == '__main__':
    rospy.init_node('imu_stabilized_to_footprint')
    
    br = tf.TransformBroadcaster()
    
    rospy.Subscriber('/imu/data_corrected',
                     sensor_msgs.msg.Imu,
                     imu_msg_callback)
    rospy.spin()