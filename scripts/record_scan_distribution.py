#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from drawnow import drawnow, figure
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
___________________________________________________________


'''
def makeFig(): # creat a function to make plot
	plt.plot(X,Y, 'b.')
	plt.axis([0, 120, -120, 120])

def makeHist(): # creat a function to make plot
	plt.subplot(121)
	n, bins, patches = plt.hist((X), 1000, normed=1, facecolor='red', edgecolor='red', alpha=0.5, range=(-120,0))
	plt.title('negative Y measurements with x<4 m')
	plt.subplot(122)
	n, bins, patches = plt.hist((Y), 1000, normed=1, facecolor='green', edgecolor='green',alpha=0.5, range=(0,120))
	plt.title('positive Y measurementss with x<4 m')
	

 

def pol2cart(theta,rho):
	x = rho * np.cos(theta)
	y = rho * np.sin(theta)
	return x, y
if __name__ == '__main__':
	rospy.init_node ('record_scan_distribution')

	
	# Let user choose bag file
	root = Tk()
	root.withdraw()
	filename = askopenfilename()

	print 'Loading bag file ...'
	bag = rosbag.Bag(filename)

	# Close Tkinter instance
	root.destroy()

	X=[]
	Y=[]
	Angles = np.linspace(np.deg2rad(-95),np.deg2rad(95),1521)
	print 'Reading scan messages ...'
	for topic, msg, t in bag.read_messages('/scan'):
		#res = [np.array([1,-2,3,4,5]), np.array([10,20,-30,40,50])]
		
		res = pol2cart(Angles,msg.ranges)
		
		inds_X = np.nonzero(res[0]<4)
		inds_XX = np.nonzero(res[0][inds_X]>0)
		inds_Ypos = np.nonzero(res[1][inds_X]>0)
		inds_Yneg = np.nonzero(res[1][inds_X]<0)
		
		#debug
		#print ('X coordinate' + str(res[0]))
		#print ('Y coordinate' + str(res[1]))
		'''
		X = (res[0][inds_X])
		Y = (res[1][inds_X])
		
		X.append(res[0][inds_XX])
		Y.append(res[1][inds_XX])
		
		
		X = (res[0][inds_X])
		Y = (res[1][inds_X])
		'''
		X = (res[0][inds_X])
		Y = (res[1][inds_X])
		
		drawnow(makeFig)
		
		
		
		#plt.show()
		
		#median_y = np.median(np.array(angleX)*180/math.pi)
	bag.close()
	
'''
# the histogram of the data
plt.subplot(121)
n, bins, patches = plt.hist(np.array(X), 1000, normed=1, facecolor='red', edgecolor='red', alpha=0.5)
plt.subplot(122)
n, bins, patches = plt.hist(np.array(Y), 1000, normed=1, facecolor='green', edgecolor='green',alpha=0.5)


plt.show()
'''
