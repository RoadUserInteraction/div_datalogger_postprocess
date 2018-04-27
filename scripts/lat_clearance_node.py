#!/usr/bin/env python
import math
import time

import rospy
import tf
import rosbag

from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

import numpy as np
import matplotlib.pyplot as plt

from hdbscan import HDBSCAN
from sklearn import metrics

from scipy.spatial import ConvexHull


def to_point_array(scan_msg):
    
    ranges = np.asarray(scan_msg.ranges)
    angles = np.linspace(-95, 95, 1521)*math.pi/180
    
    # Remove 120.0 detections
    ind_to_keep = ranges != 120.0

    ranges = ranges[ind_to_keep]
    angles = angles[ind_to_keep]

    # Remove NaN's
    idx = np.argwhere(~np.isnan(ranges))

    point_array = np.empty((len(idx), 2))
    ranges = np.array(ranges[idx])
    angles = np.array(angles[idx])

    # Remove unnecessary dimensions 
    point_array[:, 0] = np.squeeze(ranges * np.cos(angles))
    point_array[:, 1] = np.squeeze(ranges * np.sin(angles))

    return point_array

def scanCallback(scan_msg):
    global centroid_list
    global previous_stamp
    global speed_list
    global lc_list
    global num_ot
    global previous_angle
    global timer
    
    point_array = to_point_array(scan_msg)

    ot_print_result = False

    num_cars = 0

    if point_array.shape[0] > 180:
        # Perform HDB clustering
        hdb = HDBSCAN(min_cluster_size=180, metric='euclidean', min_samples=80, 
                        cluster_selection_method='eom', allow_single_cluster=True, 
                        gen_min_span_tree=True).fit(point_array)
        hdb_labels = hdb.labels_

        hdb_unique_labels = set(hdb_labels)

        hdb_colors = plt.cm.Spectral(np.linspace(0, 1, len(hdb_unique_labels)))

        markerArray = MarkerArray()

        #print "Number of clusters: " + str(len(hdb_unique_labels))
        
        for i in hdb_unique_labels:
            index = hdb_labels == i
            if np.count_nonzero(index) > 0 and i > -1:
                hull = ConvexHull(point_array[index])
                #print "Hull area: " + str(hull.area)
                
                # Marker color blue if cluster is not a car
                r = 0
                g = 0
                b = 1

                centroid = np.mean(point_array[index], axis=0)

                if point_array[index].shape[0]/hull.area > 10 and hull.area < 8:
                    # Car detected!
                    
                    # Car marker in green
                    r = 0
                    g = 1
                    b = 0

                    if len(centroid_list) > 0:
                        previous_centroid = centroid_list[-1]
                        
                        dt = scan_msg.header.stamp.to_sec() - previous_stamp.to_sec()

                        angle = math.atan2(centroid[1] - previous_centroid[1], centroid[0] - previous_centroid[0])
                        
                        angle_change = None                        

                        if len(centroid_list) > 1:
                            angle_change = angle - previous_angle
                            #print "Angle change: " + str(angle_change*180/math.pi) + "deg"

                        
                        # Check if car is overtaking or (in 'else') oncoming vehicle
                        if centroid[1] - previous_centroid[1] < 0:# and centroid[0] < 5:

                            num_cars = num_cars + 1

                            # Update lists
                            if previous_centroid[1] > 0:
                                speed_list.append(np.linalg.norm(centroid - previous_centroid) / dt * 3.6)

                            centroid_list.append(centroid)
                            lc_list.append(np.min(np.linalg.norm(point_array[index], axis=1)))
                            previous_stamp = scan_msg.header.stamp
                        else:
                            # Check if vehicle was in positive-y and has crossed x-axis
                            if (centroid_list[0][1] > 0 and previous_centroid[1] < 0): # and abs(angle_change * 180/math.pi)<80:
                                ot_print_result = True
                                print "------------- ot_print_result = true ------------------"
                                print "Number points in lat. clearance list: " + str(len(lc_list))
                                
                            # Reset lists
                            speed_list = []
                            centroid_list = []
                            previous_angle = None
                            previous_stamp = scan_msg.header.stamp
                            #print "Angle: " + str(angle*180/math.pi) + "deg"
                            #print "RESET by new vehicle ------------------------"

                        # HERE?
                        previous_angle = angle
                    else:
                        # Start new lists
                        centroid_list.append(centroid)
                        previous_stamp = scan_msg.header.stamp
                        lc_list.append(np.min(np.linalg.norm(point_array[index])))
                
                else:
                    #dt = scan_msg.header.stamp.to_sec() - previous_stamp.to_sec()
                    
                    '''
                    if dt > 1.0:
                        print "Overtaking #" + str(num_ot)
                        print "Speed: " + str(np.round_(np.mean(np.asarray(speed_list[0])), 1)) + " km/h"#, std=" + str(np.round_(np.std(np.asarray(speed_list)), 1))
                        print "LC (min): " + str(lc_min)
                        
                        speed_list = []
                        centroid_list = []
                        lc_list = []
                        previous_stamp = 0'''


                '''print "Cluster " + str(i)
                print "- Points: " + str(point_array[index].shape[0])
                print "- Density: " + str(point_array[index].shape[0]/hull.area)
                print "- Area: " + str(hull.area)
                print "- RGB: " + str([r, g, b])'''
                

                marker = Marker()
                marker.header.frame_id = scan_msg.header.frame_id
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.id = marker.id + 1
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1
                marker.color.a = 1.0
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = centroid[0]
                marker.pose.position.y = centroid[1]
                marker.pose.position.z = 0.0

                markerArray.markers.append(marker)

        '''if num_cars == 0:
            marker = Marker()
            marker.header.frame_id = scan_msg.header.frame_id
            marker.type = marker.SPHERE
            marker.action = marker.DELETEALL
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0

            markerArray.markers.append(marker)'''


        pub_cluster.publish(markerArray)

    # Time step from detection of car to current time
    if previous_stamp != None:
        timer = scan_msg.header.stamp.to_sec() - previous_stamp.to_sec()

    # Print results if OT finished or timer passed 0.5 seconds
    if (ot_print_result or timer > 0.5):
        # Check if list is not empty and lateral clearance has a reasonable value
        if lc_list and np.min(np.asarray(lc_list)) < 3:
            lc_min = np.min(np.asarray(lc_list))
            
            num_ot = num_ot + 1

            print "Overtaking # " + str(num_ot)
            print "Speed: " + str(np.round_(np.mean(np.asarray(speed_list)), 1)) + " km/h, std=" + str(np.round_(np.std(np.asarray(speed_list)), 1))
            print "LC (min): " + str(lc_min)
            print "Number points in lat. clearance list: " + str(len(lc_list))

            lc_list = []
            if timer > 0.5:
                # Reset lists
                speed_list = []
                centroid_list = []
                lc_list = []
                previous_angle = None
                previous_stamp = scan_msg.header.stamp

                print "RESET by timer ------------------------"


if __name__ == '__main__':
    rospy.init_node('cluster_node')

    # Set up global variables
    speed_list = []
    centroid_list = []
    lc_list = []
    previous_stamp = None
    previous_angle = None
    markerArray = None
    num_ot = 0
    timer = 0

    # Subscribers
    rospy.Subscriber("scan_filtered", LaserScan, scanCallback)

    # Publishers
    pub_cluster = rospy.Publisher('cluster_centroids', MarkerArray, queue_size=10)

    rospy.spin()