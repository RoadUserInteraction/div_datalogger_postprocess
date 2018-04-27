#!/usr/bin/env python
import math
import time

import rospy
import tf
import rosbag
from sensor_msgs.msg import LaserScan

import numpy as np
import matplotlib.pyplot as plt

from hdbscan import HDBSCAN
from sklearn import metrics
from sklearn.preprocessing import StandardScaler

from scipy.spatial import ConvexHull


def to_point_array(scan_msg):
    point_array = np.empty((1521, 2))

    ranges = np.asarray(scan_msg.ranges)
    angles = np.linspace(-95, 95, 1521)*math.pi/180

    point_array[:, 0] = ranges * np.cos(angles)
    point_array[:, 1] = ranges * np.sin(angles)

    return point_array


def scanCallback(scan_msg):
    point_array = to_point_array(scan_msg)

    point_array = StandardScaler().fit_transform(point_array)
    
    hdb = HDBSCAN(min_cluster_size=20).fit(point_array)
    hdb_labels = hdb.labels_

    # Black removed and is used for noise instead.
    hdb_unique_labels = set(hdb_labels)
    hdb_colors = plt.cm.Spectral(np.linspace(0, 1, len(hdb_unique_labels)))

    #print hdb_unique_labels
    for i in range(len(hdb_unique_labels)):
        index = hdb_labels == i
        
        if len(np.count_nonzero(index)):
            hull = scipy.spatial.ConvexHull(X[index])
                if hull.area < 10:
                    print "CAR!"
                    pub_cluster.publish()
    #   fig.plot(point_array[hdb_labels == k, 0], point_array[hdb_labels == k, 1], 'o', markerfacecolor=col,
    #              markeredgecolor='k', markersize=6)



fig = None

if __name__ == '__main__':
    rospy.init_node ('cluster_node')

    # Subscribers
    rospy.Subscriber("scan_filtered", LaserScan, scanCallback)

    # Publishers
    pub_cluster = rospy.Publisher('scan_clustered', LaserScan, queue_size=10)

    rospy.spin()    