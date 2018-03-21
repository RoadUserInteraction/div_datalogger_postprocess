#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy
import tf
import math

if __name__ == '__main__':
	rospy.init_node('imu_static_drift')

	directory 		= '/home/chalmers/bagfiles/imu_static_drift/renamed/'
	file_pre 		= ''
	num_split_files = 2

	time_stamp = []
	angular_velocity_x = []
	angular_velocity_y = []
	angular_velocity_z = []

	for idx in range(0, num_split_files):
		print('Loading file ' + str(idx) + '/' + str(num_split_files) + '...')
		print (directory  + str(idx) + '.bag')
		#bag = rosbag.Bag(directory + file_pre + str(idx) + '.bag')
		bag = rosbag.Bag(directory + str(idx) + '.bag')
		#print (directory + file_pre + str(idx) + '.bag')
		i = 0
		for topic, msg, t in bag.read_messages('/imu/data_raw'):
			if i == 0:			
				angular_velocity_x.append(msg.angular_velocity.x)
				angular_velocity_y.append(msg.angular_velocity.y)
				angular_velocity_z.append(msg.angular_velocity.z)
				time_stamp.append(t.to_sec())

			i = i + 1
			if i > 9:
				i = 0

			

		bag.close()

	# Mean angular velocity values
	mean_rate_x = np.mean(np.array(angular_velocity_x))
	mean_rate_y = np.mean(np.array(angular_velocity_y))
	mean_rate_z = np.mean(np.array(angular_velocity_z))
	
	print('-----------------')
	print('Mean roll rate [rad/s]: ' 	+ str(mean_rate_x))
	print('Mean pitch rate [rad/s]: ' 	+ str(mean_rate_y))
	print('Mean yaw rate [rad/s]: ' 	+ str(mean_rate_z))

	# Mean angular velocity per sec values
	zeroed_time = np.array(time_stamp) - time_stamp[0]

	half_time = zeroed_time[int(round(len(zeroed_time) / 2))]

	print('-----------------')
	print('Mean roll rate per time [rad/s/s]: ' + str(mean_rate_x / half_time))
	print('Mean roll rate per time [rad/s/s]: ' + str(mean_rate_y / half_time))
	print('Mean roll rate per time [rad/s/s]: ' + str(mean_rate_z / half_time))

	# Plotting over time
	plt.plot(zeroed_time, angular_velocity_x)
	plt.plot(zeroed_time, angular_velocity_y)
	plt.plot(zeroed_time, angular_velocity_z)
	
	plt.xlabel('Time [s]')
	plt.ylabel('Angular velocity [rad/s]')
	plt.legend(['roll', 'pitch', 'yaw'])
	plt.title('IMU static drift')
	plt.show()

