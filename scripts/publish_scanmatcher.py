#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import rospy
import tf
import math
from Tkinter import Tk
from tkFileDialog import askopenfilename
import os
import shutil


def quaternion_mult(q, r):
    return [r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
            r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
            r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
            r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]]


def unity_euler_to_quaternion(x, y, z):
    return quaternion_mult(quaternion_mult(euler_to_quaternion(0, y, 0),
                                           euler_to_quaternion(x, 0, 0)),
                           euler_to_quaternion(0, 0, z))


if __name__ == '__main__':
    rospy.init_node('publish_scanmatcher')

    print 'Loading bag file ...'
    #filename = '/home/chalmers/bagfiles/Marco_Pilot_PreTest/2018-03-20-09-55-28_0.bag'
    filename = '/home/chalmers/bagfiles/Marco_Pilot_PreTest/2018-03-20-09-56-34_0.bag'
    bag = rosbag.Bag(filename)

    #out_bag = rosbag.Bag('/home/chalmers/bagfiles/gyro_correction/2018-03-13-14-29-50_0_CORRECT.bag')

    #limit_quaternion = unity_euler_to_quaternion()
    print 'Reading IMU messages ...'
    angleX = []
    angleY = []
    angleZ = []
    angle = []
    theta = []
    theta_n = []
    ux,uy,uz = [],[],[]

    for topic, msg, t in bag.read_messages('/imu/data'):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        q_c = [msg.orientation.w, -msg.orientation.x, -msg.orientation.y, -msg.orientation.z]

        r = quaternion_mult(q, quaternion_mult([0, 0, 0, 1],q_c))

        if euler[0] > 0:
            angleX.append((euler[0] - math.pi)* 180 / math.pi)
        else:
            angleX.append((euler[0] + math.pi) * 180 / math.pi)
        angleY.append((euler[1])* 180 / math.pi)
        if euler[2] > 0:
            angleZ.append((euler[2] - math.pi) * 180 / math.pi)
        else:
            angleZ.append((euler[2] + math.pi) * 180 / math.pi)


        angle.append(math.acos(r[3])*180/math.pi)
        theta.append((math.acos(q[0])*360/math.pi)-180) 
        #theta_n.append(math.acos(math.sqrt(q[1]**2+q[2]**2+q[3]**2))*360/math.pi)
        theta_n.append((math.atan2(math.sqrt(q[1]**2+q[2]**2+q[3]**2),q[0])*360/math.pi)-180)

        print (math.acos(q[0])*360/math.pi)-180, (math.acos(math.sqrt(q[1]**2+q[2]**2+q[3]**2))*180/math.pi)
        ux.append((1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[1])
        uy.append((1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[2])
        uz.append((1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[3])

        print '---'
        print ((1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[1],(1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[2],(1/math.sqrt(q[1]**2+q[2]**2+q[3]**2))*q[3])
        print '---'
    bag.close()


    print('Plotting histograms of record'+ filename) 
    plt.figure(1)
    
    plt.subplot(331)
    n, bins, patches = plt.hist(np.array(angleX), 1000, density=1, facecolor='red', edgecolor='red',
                                alpha=0.5)
    plt.xlabel('Euler[0]')

    plt.subplot(332)
    n, bins, patches = plt.hist(np.array(angleY), 1000, density=1, facecolor='green',
                                edgecolor='green', alpha=0.5)
    plt.xlabel('Euler[1]')

    plt.subplot(333)
    n, bins, patches = plt.hist(np.array(angleZ), 1000, density=1, facecolor='blue',
                                edgecolor='blue', alpha=0.5)
    plt.xlabel('Euler[2]')
    

    plt.subplot(334)
    plt.plot((angleX),color='red')
    plt.xlabel('Euler[0]')

    plt.subplot(335)
    plt.plot((angleY),color='green')
    plt.xlabel('Euler[1]')

    plt.subplot(336)
    plt.plot((angleZ),color='blue')
    plt.xlabel('Euler[2]')

    plt.subplot(337)
    plt.plot(ux)
    plt.xlabel('ux')

    plt.subplot(338)
    plt.plot(uy)
    plt.xlabel('uy')

    plt.subplot(339)
    plt.plot(uz)
    plt.xlabel('uz')

    

    plt.figure(2)
    plt.plot(theta_n)
    plt.plot(angle)
    plt.plot(theta)
    plt.xlabel('rotation angle from quaternion')

    plt.show()


    print 'Exiting ...'