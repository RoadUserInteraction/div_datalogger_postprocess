#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy
import tf
import math
from Tkinter import Tk
from tkFileDialog import askopenfilename

if __name__ == '__main__':
	rospy.init_node('record_visualization')

	# Let user choose bag file
	root = Tk()
	root.withdraw()
	filename = askopenfilename()

	print 'Loading bag file ...'
	bag = rosbag.Bag(filename)

	# Close Tkinter instance
	root.destroy()

	angleX=[]
	angleY=[]
	angleZ=[]
	ang_velocity_X=[]
	ang_velocity_Y=[]
	ang_velocity_Z=[]

	print 'Reading IMU messages ...'
	for topic, msg, t in bag.read_messages('/imu/data'):
		quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		if euler[0]>0:
			angleX.append(euler[0] - math.pi)
		else: 
			angleX.append(euler[0] + math.pi)
		
		if euler[2]>0:
			angleZ.append(euler[2] - math.pi)
		else: 
			angleZ.append(euler[2] + math.pi)
		
		angleY.append(euler[1])

		ang_velocity_X.append(msg.angular_velocity.x)
		ang_velocity_Y.append(msg.angular_velocity.y)
		ang_velocity_Z.append(msg.angular_velocity.z)
		
	bag.close();

	mean_ang_x = np.mean(np.array(angleX)*180/math.pi)
	mean_ang_y = np.mean(np.array(angleY)*180/math.pi)
	mean_ang_z = np.mean(np.array(angleZ)*180/math.pi)
	
	print 'Mean Roll is ', mean_ang_x
	print 'Mean Pitch is ', mean_ang_y
	print 'Mean Yaw is ', mean_ang_z

	mean_vel_x = np.mean(np.array(ang_velocity_X))
	mean_vel_y = np.mean(np.array(ang_velocity_Y))
	mean_vel_z = np.mean(np.array(ang_velocity_Z))

	print 'Mean vel_X is ', mean_vel_x
	print 'Mean vel_Y is ', mean_vel_y
	print 'Mean vel_Z is ', mean_vel_z


	# Histograms of orientation data
	
	print 'Plotting histograms ...'
	
	plt.subplot(231)
	n, bins, patches = plt.hist(np.array(angleX)*180/math.pi, 1000, density=1, facecolor='red', edgecolor='red', alpha=0.5)
	plt.xlabel('Euler[0]')
	
	plt.subplot(232)
	n, bins, patches = plt.hist(np.array(angleY)*180/math.pi, 1000, density=1, facecolor='green', edgecolor='green',alpha=0.5)
	plt.xlabel('Euler[1]')

	plt.subplot(233)
	n, bins, patches = plt.hist(np.array(angleZ)*180/math.pi, 1000, density=1, facecolor='blue', edgecolor='blue',alpha=0.5)
	plt.xlabel('Euler[2]')

	plt.subplot(234)
	n, bins, patches = plt.hist(np.array(ang_velocity_X)*1, 1000, density=1, facecolor='orange', edgecolor='orange', alpha=0.5)
	plt.xlabel('Mean vel_X')
	
	plt.subplot(235)
	n, bins, patches = plt.hist(np.array(ang_velocity_Y)*1, 1000, density=1, facecolor='olive', edgecolor='olive',alpha=0.5)
	plt.xlabel('Mean vel_Y ')

	plt.subplot(236)
	n, bins, patches = plt.hist(np.array(ang_velocity_Z)*1, 1000, density=1, facecolor='cyan', edgecolor='cyan',alpha=0.5)
	plt.xlabel('Mean vel_Z')
	plt.show()

	print 'Exiting ...'