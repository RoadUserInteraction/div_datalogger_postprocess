"""
===========
Random data
===========

An animation of random data.

"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag
import rospy
import time

start_time = None
fig, ax = plt.subplots()
polar_line, = plt.polar([], [], animated=True)
bag = rosbag.Bag("test_corrected.bag")
theta = np.zeros([0, 0])

def init():
    ax.set_xlim(0, 2 * np.pi)
    ax.set_ylim(-1, 1)
    return polar_line,

def animate(i):
    global start_time, theta
    for topic, msg, t in bag.read_messages(topics=["/scan_corrected"], start_time=start_time):
        if theta.size == 0:
            theta = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        rho = np.asarray(msg.ranges)
        polar_line.set_data(theta, rho)
        if start_time is not None:
            time.sleep(t.to_sec() - start_time.to_sec() + rospy.Duration(0, 1).to_sec())

        start_time = t + rospy.Duration(0, 1)
    return polar_line,



ani = animation.FuncAnimation(fig, animate, init_func=init, repeat=False, interval=0, blit=True)
plt.show()