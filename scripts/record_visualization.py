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
		
	bag.close();

	mean_ang_x = np.mean(np.array(angleX)*180/math.pi)
	mean_ang_y = np.mean(np.array(angleY)*180/math.pi)
	mean_ang_z = np.mean(np.array(angleZ)*180/math.pi)
	
	print 'Mean Roll is ', mean_ang_x
	print 'Mean Pitch is ', mean_ang_y
	print 'Mean Yaw is ', mean_ang_z

	# Histograms of orientation data
	print 'Plotting histograms ...'
	plt.subplot(131)
	n, bins, patches = plt.hist(np.array(angleX)*180/math.pi, 1000, density=1, facecolor='red', edgecolor='red', alpha=0.5)
	plt.subplot(132)
	n, bins, patches = plt.hist(np.array(angleY)*180/math.pi, 1000, density=1, facecolor='green', edgecolor='green',alpha=0.5)
	plt.subplot(133)
	n, bins, patches = plt.hist(np.array(angleZ)*180/math.pi, 1000, density=1, facecolor='blue', edgecolor='blue',alpha=0.5)

	plt.show()

	print 'Exiting ...'