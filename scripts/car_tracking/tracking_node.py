#!/usr/bin/env python
import math
import time

import rospy
import tf
import rosbag
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

import numpy as np
import matplotlib.pyplot as plt

import pcl
import sensor_msgs.point_cloud2 as pc2

from laser_geometry import LaserProjection


laserProjection = None
pub_cloud_1     = None
pub_cluster_1   = None


def getPCLFromROScloud(ros_cloud):
    pc = pc2.read_points(ros_cloud, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for pcl_cloud in pc:
        pc_list.append([pcl_cloud[0], pcl_cloud[1], pcl_cloud[2]])

    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(pc_list)

    return pcl_cloud

def getROSFromPCLcloud(pcl_cloud, header):
    ros_cloud = np.zeros([pcl_cloud.size, 3], dtype=np.float32)
    for i, pt in enumerate(pcl_cloud):
        ros_cloud[i] = [pt[0], pt[1], pt[2]]

    ros_cloud = pc2.create_cloud_xyz32(header, ros_cloud)

    return ros_cloud

def scanCallback(scan_in):
    # Project laser to point cloud
    cloud_1 = laserProjection.projectLaser(scan_in)
    #pub_cloud_1.publish(cloud_1)

    # Convert to pcl cloud
    p = getPCLFromROScloud(cloud_1)

    # TODO: TRANSFORM BACK TO LASER FRAME
    # z limiting
    '''z_filter = p.make_passthrough_filter()
    z_filter.set_filter_field_name("z")
    z_filter.set_filter_limits(-1.2, 4)
    p = z_filter.filter()'''

    y_filter = p.make_passthrough_filter()
    y_filter.set_filter_field_name("y")
    y_filter.set_filter_limits(-100.0, 100.0)
    p = y_filter.filter()

    x_filter = p.make_passthrough_filter()
    x_filter.set_filter_field_name("x")
    x_filter.set_filter_limits(0, 8.0)
    p = x_filter.filter()


    # TODO: Limit z axis from base_footprint frame!

    # Statistical outlier removal
    stat_filter = p.make_statistical_outlier_filter()
    stat_filter.set_mean_k(50)
    stat_filter.set_std_dev_mul_thresh(1.0)
    p = stat_filter.filter()

    # Convert back to PointCloud2 message
    cloud_msg = getROSFromPCLcloud(p, scan_in.header)

    # Publish cloud
    pub_cloud_1.publish(cloud_msg)

'''
    # Cluster point cloud
    objects = ph.XYZRGB_to_XYZ(objects)
    tree = objects.make_kdtree()
    segment = p.make_EuclideanClusterExtraction()
    segment.set_ClusterTolerance(0.02)
    segment.set_MinClusterSize(100)
    segment.set_MaxClusterSize(25000)
    segment.set_SearchMethod(tree)
    cluster_indices = segment.Extract()

    cloud_cluster = pcl.PointCloud()

    cloud_2 = np.zeros([cloud_cluster.size, 3], dtype=np.float32)
    for i, pt in enumerate(cloud_cluster):
        cloud_2[i] = [pt[0], pt[1], pt[2]]

    cloud_msg = pc2.create_cloud_xyz32(scan_in.header, cloud_2)

    # Publish cloud
    pub_cluster_1.publish(cloud_msg)
'''

if __name__ == '__main__':
    rospy.init_node ('tracking_node')

    # Laser geometry
    laserProjection = LaserProjection()

    # Subscribers
    rospy.Subscriber("scan", LaserScan, scanCallback)

    # Publishers
    pub_cloud_1 = rospy.Publisher('cloud_1', PointCloud2, queue_size=10)
    pub_cluster_1 = rospy.Publisher('cluster_1', PointCloud2, queue_size=10)

    rospy.spin()    