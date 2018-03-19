'''
<!-- to be connverted in roslaunch script
  rosrun topic_tools transform /imu/data_raw /imu/data_raw_corrected geometry_msgs/Vector3 '[m.angular_velocity.x-0.0283588610772, m.angular_velocity.y+0.0100331115389, m.angular_velocity.z-0.2466768508]' --import sensor_msgs.msg

    rosrun topic_tools transform /imu/data_raw /imu/data_raw_corrected sensor_msgs/Imu '[m.header, m.orientation, [m.angular_velocity.x-0.0283588610772, m.angular_velocity.y+0.0100331115389, m.angular_velocity.z-0.2466768508], m.linear acceleration]' --import sensor_msgs.msg

    rosrun topic_tools transform /imu/data_raw imu/data_raw_corrected sensor_msgs/Imu '[m.header.seq,m.header.stamp,m.header.frame_id.replace("base_link","new_frame"),m.orientation,m.orientation_covariance,m.angular_velocity,m.angular_velocity_covariance,m.linear_acceleration,m.linear_acceleration_covariance]'

/////////
 rosrun topic_tools transform /imu/data_raw/angular_velocity /imu/data_raw_corrected/angular_velocity geometry_msgs/Vector3 '[m.x-0.0283588610772, m.y+0.0100331115389, m.z-0.2466768508]' 

rosrun topic_tools transform /imu/data_raw/header /imu/data_raw_corrected/header std_msgs/Header '[m.seq,m.stamp,m.frame_id.replace("base_link","new_frame")]'


rosrun topic_tools transform /imu/data_rawimu/data_raw_corrected sensor_msgs/Imu '[[m.header.seq,m.header.stamp,m.header.frame_id.replace("base_link","new_frame")],[m.orientation],[],[],[],[],[]]''


--> 
'''
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
	bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-14-29-50_0.bag')#torso
	
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
	bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-14-29-50_0_test.bag', 'a') 
	DataAll=[]
	
	for topic, msg, t in bag.read_messages('/imu/data_raw'):
		DataAll.append(msg)
		DataAll.append(t)
	print DataAll
	bag.close()
	
	bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-14-29-50_0_test2.bag', 'a') 
	for msg, t in DataAll:
		msg_corrected = msg
		msg_corrected.header.frame_id = "base_link_corrected"
		msg_corrected.angular_velocity.x = msg.angular_velocity.x - mean_vel_x
		msg_corrected.angular_velocity.y = msg.angular_velocity.y - mean_vel_y
		msg_corrected.angular_velocity.z = msg.angular_velocity.z - mean_vel_z
		bag.write('/imu/data_raw_corrected', msg_corrected, t)

	bag.close()
	