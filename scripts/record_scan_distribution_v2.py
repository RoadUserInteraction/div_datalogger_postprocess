#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from Tkinter import Tk
from tkFileDialog import askopenfilename


'''
 script to show distribution of laser points in x-y coordinates
____________________________________________________________
												   -|---->y
A 													|		B
													|
 _________   _________   _________    _________   __+x______
		+ x
		|     <-----Walking direction
		|
 y<-----|--   
____________________________________________________________

'''
	
def pol2cart(theta,rho):
	x = rho * np.cos(theta)
	y = rho * np.sin(theta)
	return (x, y)
	
if __name__ == '__main__':
	rospy.init_node ('record_scan_distribution_v2')

	# Let user choose bag file
	root = Tk()
	root.withdraw()
	filename = askopenfilename()

	print 'Loading bag file ...'
	bag = rosbag.Bag(filename)

	# Close Tkinter instance
	root.destroy()

	y = np.array([])

	angles = np.linspace(np.deg2rad(-95), np.deg2rad(95), 1521)

	idx = 0
	print 'Reading scan messages ...'
	for topic, msg, t in bag.read_messages('/scan'):
		if idx == 0:			
			x_cart, y_cart = pol2cart(angles, msg.ranges)

			# Filter out scan locations within 4 m in x-direction
			inds_x = np.nonzero(x_cart < 4)

			y = np.append(y, np.array(y_cart[inds_x]), axis=0)

		idx = idx + 1
		if idx > 9:
			idx = 0

	bag.close()
		
	print 'Plotting histogram ...'
	n, bins, patches = plt.hist(y, 1000, density=1, facecolor='blue', edgecolor='blue', alpha=0.5, range=(-120,120))
	plt.title('y-position distribution of LiDAR detections with x < 4 m')
	plt.xlabel('y-position [m]')
	plt.ylabel('PDF [1/m]')
	plt.show()

	print 'Exiting ...'