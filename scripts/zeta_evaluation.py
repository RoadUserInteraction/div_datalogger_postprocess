#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy

import math



if __name__ == '__main__':
	

	M=[[0.00086029042266,-0.000230406322071,-3.47646385229e-05], #0
	[0.00081417750516,-0.000317978558728,0.000189661962],
	[0.000806892708922,-0.000326136061248,-0.000258489721433],
	[0.000472904387065,-0.000298130260223,-0.00044017427632], #3
	[0.000491665498907,-0.000114572503527,-0.000567593559805], #4
	[0.000363410225473,-0.000136593139566,-0.000575495950403], #5
	[0.000232356839046,2.04473805957e-06,-0.00022395332569], #6
	[-0.000114323072393,1.20436556321e-05,-0.000183071992035], #7
	[0.000208383698204,-9.48830952312e-05,-6.30097627404e-05],#8
	[0.000175192486276,-0.000231085262435,-0.00015047817424], #9
	[4.93367152826e-05,-9.08944653063e-05,-0.000146078252098], #10
	[-8.33453816114e-05,-2.51741172941e-05,0.000151342343192],#11
	[-0.000316701526173,-5.54901775585e-05,7.74011872544e-05]] #12
	m = np.array(M)
	T = np.linspace(1,len(m[:,0]),len(m[:,0]))
	
	x = np.mean(m[:,0])
	x_max= np.max(m[:,0])
	x_min= np.min(m[:,0])
	y = np.mean(m[:,1])
	z = np.mean(m[:,2])
	print 'Mean vel_X is ', x
	print 'Delta_vel_X is ', (x_max-x_min)
	print 'Mean vel_Y is ', y
	print 'Mean vel_Z is ', z


	#for i in range(len(m[0,:]+1)):
		# Plotting over time
	plt.plot(T, m[:,0])
	plt.plot(T, m[:,1])
	plt.plot(T, m[:,2])
	
	plt.xlabel('Time [s]')
	plt.ylabel('Angular velocity [rad/s]')
	plt.legend(['roll [0]', 'pitch[1]', 'yaw[2]'])
	plt.title('IMU static drift')
	plt.show()
'''
Mean roll rate [rad/s]: 0.0003564204244264037
Mean pitch rate [rad/s]: -0.00015680023358947905
Mean yaw rate [rad/s]: -0.00019800415231594235
-----------------
Mean roll rate per time [rad/s/s]: 1.5078431993692764e-08
Mean roll rate per time [rad/s/s]: -6.633462890290391e-09
Mean roll rate per time [rad/s/s]: -8.376602294802573e-09

Creates a list containing 5 lists, each of 8 items, all set to 0

rospy.init_node('imu_static_drift')

	directory 		= '/home/ubuntu/bagfiles/imu_static_drift/'
	file_pre 		= '2018-03-20-09-55-28_'
	num_split_files = 12

	time_stamp = []
	angular_velocity_x = []
	angular_velocity_y = []
	angular_velocity_z = []

	for idx in range(0, num_split_files):
		print('Loading file ' + str(idx + 1) + '/' + str(num_split_files) + '...')

		bag = rosbag.Bag(directory + file_pre + str(idx) + '.bag')

		for topic, msg, t in bag.read_messages('/imu/data_raw'):
			angular_velocity_x.append(msg.angular_velocity.x)
			angular_velocity_y.append(msg.angular_velocity.y)
			angular_velocity_z.append(msg.angular_velocity.z)
			time_stamp.append(t.to_sec())

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



0			
Mean Roll is  0.161116370346
Mean Pitch is  0.6756685564
Mean Yaw is  58.349004874
Mean vel_X is  0.00086029042266
Mean vel_Y is  -0.000230406322071
Mean vel_Z is  -3.47646385229e-05

1
Mean Roll is  0.171859238482
Mean Pitch is  0.695361041996
Mean Yaw is  58.3300887368
Mean vel_X is  0.00081417750516
Mean vel_Y is  -0.000317978558728
Mean vel_Z is  0.00018966
2
Mean Roll is  0.174755419909
Mean Pitch is  0.703927700624
Mean Yaw is  58.3260174628
Mean vel_X is  0.000806892708922
Mean vel_Y is  -0.000326136061248
Mean vel_Z is  -0.000258489721433
3
Mean Roll is  0.185813797079
Mean Pitch is  0.709265183552
Mean Yaw is  58.3722793644
Mean vel_X is  0.000472904387065
Mean vel_Y is  -0.000298130260223
Mean vel_Z is  -0.00044017427632

4
Mean Roll is  0.184846050045
Mean Pitch is  0.714450668355
Mean Yaw is  58.3763347764
Mean vel_X is  0.000491665498907
Mean vel_Y is  -0.000114572503527
Mean vel_Z is  -0.000567593559805
4filtering
Mean vel_X is  0.000492022793297
Mean vel_Y is  -0.000114419237267
Mean vel_Z is  -0.000567770287357

5
Mean Roll is  0.184966047558
Mean Pitch is  0.71833711828
Mean Yaw is  58.3472978653
Mean vel_X is  0.000363410225473
Mean vel_Y is  -0.000136593139566
Mean vel_Z is  -0.000575554269568
5raw
Mean Roll is  180.0
Mean Pitch is  0.0
Mean Yaw is  180.0
Mean vel_X is  0.00036345767466
Mean vel_Y is  -0.000136629813404
Mean vel_Z is  -0.000575495950403
6
Mean Roll is  0.190073751838
Mean Pitch is  0.714094665675
Mean Yaw is  58.3402517228
Mean vel_X is  0.000232356839046
Mean vel_Y is  2.04473805957e-06
Mean vel_Z is  -0.00022395332569

7
Mean Roll is  0.191449574976
Mean Pitch is  0.716751231189
Mean Yaw is  58.3376786024
Mean vel_X is  -0.000114323072393
Mean vel_Y is  1.20436556321e-05
Mean vel_Z is  -0.000183071992035

8
Mean Roll is  0.19376568022
Mean Pitch is  0.71814098727
Mean Yaw is  58.3509714026
Mean vel_X is  0.000208383698204
Mean vel_Y is  -9.48830952312e-05
Mean vel_Z is  -6.30097627404e-05


9
Mean Roll is  0.191723407662
Mean Pitch is  0.720268092569
Mean Yaw is  58.3780495004
Mean vel_X is  0.000175192486276
Mean vel_Y is  -0.000231085262435
Mean vel_Z is  -0.00015047817424



10
Mean Roll is  0.195834510425
Mean Pitch is  0.7220818347
Mean Yaw is  58.6435316708
Mean vel_X is  4.93367152826e-05
Mean vel_Y is  -9.08944653063e-05
Mean vel_Z is  -0.000146078252098



11
Mean Roll is  0.198705097366
Mean Pitch is  0.717469948047
Mean Yaw is  58.4401873966
Mean vel_X is  -8.33453816114e-05
Mean vel_Y is  -5.54901775585e-05
Mean vel_Z is  7.74011872544e-05


12
Mean Roll is  0.189929848712
Mean Pitch is  0.723499230807
Mean Yaw is  58.3882964336
Mean vel_X is  -0.000316701526173
Mean vel_Y is  -2.51741172941e-05
Mean vel_Z is  0.000151342343192

'''