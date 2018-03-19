#!/usr/bin/env python
import rospy
import tf
import math
import rosbag
import sensor_msgs.msg
import time
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
	rospy.init_node ('correct_gyro_drift')

	# Get static test for mean angular velocities for correction
	#bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-13-22-28_0.bag')#torso
	bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-15-13-59_0.bag')#torso

	
	angleX=[]
	angleY=[]
	angleZ=[]
	for topic, msg, t in bag.read_messages('/imu/data_raw'):
		angleX.append(msg.angular_velocity.x)
		angleY.append(msg.angular_velocity.y)
		angleZ.append(msg.angular_velocity.z)

	bag.close();

	mean_vel_x = np.mean(np.array(angleX))
	mean_vel_y = np.mean(np.array(angleY))
	mean_vel_z = np.mean(np.array(angleZ))
	print mean_vel_x
	print mean_vel_y
	print mean_vel_z


	# Correct raw data
	bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-15-13-59_0_test.bag', 'a') 

	for topic, msg, t in bag.read_messages('/imu/data_raw'):
		msg_corrected = msg
		msg_corrected.header.frame_id = 'base_link_corrected'
		msg_corrected.angular_velocity.x = msg.angular_velocity.x - mean_vel_x
		msg_corrected.angular_velocity.y = msg.angular_velocity.y - mean_vel_y
		msg_corrected.angular_velocity.z = msg.angular_velocity.z - mean_vel_z
		bag.write('/imu/data_raw_corrected', msg_corrected, t)

	bag.close()
	