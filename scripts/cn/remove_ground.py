import numpy as np
import rosbag
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
import math
import pcl



def get_yaw_rd(imu_rot):
    v = [1, 0, 0]
    v_ = imu_rot.rotate(v)  # get the magnitude of orientation
    w = np.cross(v, v_)  # get the sign of orientation
    yaw = np.sign(-w[2]) * np.arccos(v_[0])  #
    return yaw

def to_pcl_array(scan_msg, imu_rot):
    yaw = -get_yaw_rd(imu_rot)
    pcl_array = np.empty((1521, 3), dtype=np.float32)
    ranges = np.asarray(scan_msg.ranges)
    ind_to_keep = ranges != 120


    laser_ori_wrt_direction = -42./180*math.pi # for pedestrian dataset
    #laser_ori_wrt_direction = -108. / 180 * math.pi # for the bike dataset
    pcl_array[:, 0] = ranges * np.cos(angles + laser_ori_wrt_direction + yaw)
    pcl_array[:, 1] = ranges * np.sin(angles + laser_ori_wrt_direction + yaw)
    pcl_array[:, 2] = 0

    # remove the points outside the road
    ind_to_keep = np.logical_and(ind_to_keep, np.logical_and(0 < pcl_array[:, 1], pcl_array[:, 1] < 8))
    #ind_to_keep = np.logical_and(ind_to_keep, np.logical_and(-.8 < pcl_array[:, 1], pcl_array[:, 1] < 5)) # for bike
    return pcl_array[[ind_to_keep]]


def plot_pcl(p, p_= []):
    pcl_array = p.to_array()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pcl_array[:, 0], pcl_array[:, 1], pcl_array[:, 2])
    if p_:
        pcl_array = p_.to_array()
        ax.scatter(pcl_array[:, 0], pcl_array[:, 1], pcl_array[:, 2], c='r')
    plt.axis('equal')
    plt.show()

bag = rosbag.Bag("/home/chalmers/Desktop/2018-PedestrianOvertaking/SecondCollection/2018-03-21-12-48-27_0.bag")
#bag = rosbag.Bag("./test2/sync_filtered_test.bag")
norm_acc = []
x, y, z = [0], [0], [0]
# for pedestrian dataset 1./250, for bike 1./100
dt = 1./250
v_z = [0, 0, 1]
angle = []
angles = np.linspace(-95, 95, 1521)*math.pi/180.
curr_pcl = pcl.PointCloud()
plot_scan = False



plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
scatter1 = ax.scatter([], [], c='b')
scatter2 = ax.scatter([], [], c='r')
range = 50
plt.xlim(-range, range)
plt.ylim(-range, range)
ax.plot(0, 0, 'r+')
Pcl_ = pcl.PointCloud()
data = np.empty((0, 3), dtype=np.float64)
i = 0
# for the pedestrian dataset /imu/data and /scan
# for bike data set /imu0/data and /scan_corrected
for topic, msg, t in bag.read_messages(topics=["/imu/data", "/scan"], start_time=rospy.Time(bag.get_start_time())+rospy.Duration(240)):
    if topic == "/imu/data":
        curr_ori = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        '''acc = curr_ori.conjugate.rotate([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        x.append(acc[0]*dt+x[-1])
        y.append(acc[1]*dt+y[-1])
        z.append((acc[2] - 9.8)*dt+z[-1])'''
        v_z_ = curr_ori.rotate(v_z)
        if np.arccos(v_z_[2]) - math.pi > -.04:
            plot_scan = True
        else:
            plot_scan = False

    else:
        #if plot_scan:

        #data = np.concatenate((data, to_pcl_array(msg, curr_ori)), axis=0)
        data = to_pcl_array(msg, curr_ori)
        
        '''filter_ = Pcl_.make_statistical_outlier_filter()
        filter_.set_mean_k(50)
        filter_.set_std_dev_mul_thresh(1.0)
        Pcl_ = filter_.filter()'''

        if data.size > 1:
            Pcl_.from_array(data)

            #scatter1.set_offsets(np.c_[data[:, 0], data[:, 1]])

            seg = Pcl_.make_segmenter()
            seg.set_model_type(pcl.SACMODEL_LINE)
            seg.set_method_type(pcl.SAC_RANSAC)
            seg.set_distance_threshold(0.1)
            seg.set_optimize_coefficients(False)
            inliers, model = seg.segment()

            scatter1.set_offsets(np.c_[data[:, 0], data[:, 1]])
            dist = np.linalg.norm([np.max(data[inliers, 0]) - np.min(data[inliers, 0]), np.max(data[inliers, 1]) - np.min(data[inliers, 1])])

            if model[4] > 0.2:
                scatter2.set_offsets(np.c_[data[inliers, 0], data[inliers, 1]])
            else:
                print dist
                print model[4]
                print "------------------"
                scatter2.set_offsets(np.c_[[], []])


        #print model
        fig.canvas.draw_idle()
        plt.pause(0.01)

        '''i += 1
        if i > 10:
            data = np.empty((0, 3), dtype=np.float64)
            i = 0
        curr_pcl.from_array(to_pcl_array(msg))
        plot_pcl(curr_pcl)'''

plt.plot(angle)
'''plt.plot(y)
plt.plot(z)'''
plt.show()

print float(np.sum(np.asarray(angle) > -.05)) / len(angle)