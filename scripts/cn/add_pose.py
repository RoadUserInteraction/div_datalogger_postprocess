import rosbag
import numpy as np
from pyquaternion import Quaternion
import math
import matplotlib.pyplot as plt
import seaborn
seaborn.set()
import pcl
import rospy
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion as Quat
from geometry_msgs.msg import Point as Pt
from shutil import copyfile

def get_yaw_quaternion(imu_rot):
    v = [1, 0, 0]
    v_ = imu_rot.rotate(v)  # get the magnitude of orientation
    w = np.cross(v, v_)  # get the sign of orientation
    yaw = np.sign(-w[2]) * np.arccos(v_[0])  #
    return Quaternion(axis=[0, 0, 1], radians=yaw)

def matrix4_to_rot_trans(imu_rot, transf, mode='3d'):
    tr = transf[0:3, 3] # translation vector (x, y, z), z should be zero
    q = Quaternion(matrix=transf)  # quaternion from matrix

    if mode == '2d':
        q = get_yaw_quaternion(imu_rot)
        tr[2] = 0

    return q, tr

def get_cov(transf, orig_pcl, transf_pcl):
    q, tr = matrix4_to_rot_trans(transf)

    # get the yaw angle
    v = Quaternion(0, 1, 0, 0)
    v_ = q * v * q.conjugate
    sinRot = np.sin(np.acos(v_[1]))
    cosRot = np.cos(np.acos(v_[1]))

    cov = [0] * 36

    currPoint = orig_pcl.to_array()
    transformed_curr_point = transf_pcl.to_array()
    rotDeriv = ((-sinRot * currPoint[:, 0] - cosRot * currPoint[:, 1]) * transformed_curr_point[:, 0] + (
            cosRot * currPoint[:, 0] - sinRot * currPoint[:, 1]) * transformed_curr_point[:, 1])

    cov[0] = np.sum(transformed_curr_point[:, 0] ** 2)
    cov[7] = np.sum(transformed_curr_point[:, 1] ** 2)
    cov[35] = np.sum(rotDeriv ** 2)

    cov[1] = np.sum(transformed_curr_point[:, 0] * transformed_curr_point[:, 1])
    cov[5] = np.sum(transformed_curr_point[:, 0] * rotDeriv)
    cov[11] = np.sum(transformed_curr_point[:, 1] * rotDeriv)

    cov[6] = cov[1]
    cov[30] = cov[5]
    cov[31] = cov[11]

    return q, tr, cov

def plot_pcl(p, p_= []):
    pcl_array = p.to_array()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pcl_array[:, 0], pcl_array[:, 1], pcl_array[:, 2])
    if p_:
        pcl_array = p_.to_array()
        ax.scatter(pcl_array[:, 0], pcl_array[:, 1], pcl_array[:, 2], c='r')
    plt.show()

def to_pcl_array(scan_msg):
    pcl_array = np.empty((1521, 3), dtype=np.float32)
    ranges = np.asarray(scan_msg.ranges)
    ind_to_keep = ranges != 120
    pcl_array[:, 0] = ranges * np.cos(angles)
    pcl_array[:, 1] = ranges * np.sin(angles)
    pcl_array[:, 2] = 0

    return pcl_array[[ind_to_keep]]

def get_correct_index(bag):

    d_z = []
    v_z = Quaternion(0, 0, 0, 1)
    # orientation of the imu frame
    imu_frame = Quaternion(-0.0198769629216, 0.974536168168, -0.218396560265, -0.0467665023439)
    time = []

    for topic, msg, t in bag.read_messages(topics='/imu0/data',
                                           start_time=rospy.Time(bag.get_start_time())+rospy.Duration(0)):
        time.append(msg.header.stamp.to_sec())
        q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        v_z_ = q*imu_frame*v_z*imu_frame.conjugate*q.conjugate
        d_z.append(v_z_[3])

    angles = np.arccos(d_z)-math.pi
    return angles > -.02


def get_interpolated_quat(interp_time, scan_time, q_array):
    q_interp_array = []
    for i in range(0, len(scan_time)-1):
        # count the number of quaternion
        n = np.sum(np.logical_and(interp_time >= scan_time[i], interp_time < scan_time[i+1]))
        for j in range(1, int(n)+1):
            q_interp_array.append(Quaternion.slerp(q_array[i], q_array[i+1], float(j)/(n+1)))
    q_interp_array.append(q_array[-1])
    return q_interp_array

def write_pose_to_bag(bag, interp_time, interp_ros_time, ori, tr):
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.pose.covariance = [0] * 36
    pose_msg.header.frame_id = 'base_footprint'
    for i in range(0, interp_time.size):
        pose_msg.header.stamp = rospy.Time(interp_time[i])
        pose_msg.pose.pose.position = Pt(tr[i, 0], tr[i, 1], tr[i, 2])
        pose_msg.pose.pose.orientation = Quat(ori[i][1], ori[i][2], ori[i][3], ori[i][0])
        bag.write('/scan_pose', pose_msg, t=rospy.Time(interp_ros_time[i]))


def add_pose_estimation(bag):
    v_z = Quaternion(0, 0, 0, 1)
    # orientation of the imu frame
    imu_frame = Quaternion(-0.0198769629216, 0.974536168168, -0.218396560265, -0.0467665023439)
    x_vec = np.array([1, 0, 0])
    time = []
    previous_pcl = pcl.PointCloud()
    current_pcl = pcl.PointCloud()
    est_pose_bool = False
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = 'base_footprint'
    previous_pose = np.array([])
    previous_ori = []
    x = []
    y = []
    prev_tr = np.array([])
    cov = [0] * 36
    scan_time = []
    ros_time = []
    f_pose = 20 # Hz
    queued_pcl_list = []
    dt = 0
    print 'Extracting poses using ICP'
    for topic, msg, t in bag.read_messages(topics=['/imu0/data', '/scan_corrected'],
                                           start_time=rospy.Time(bag.get_start_time())+rospy.Duration(0)):
        #
        if topic == '/imu0/data':
            q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            v_z_ = q * imu_frame * v_z * imu_frame.conjugate * q.conjugate
            imu_rot = q
            if np.arccos(v_z_[3])-math.pi > -.05:
                est_pose_bool = True
            else:
                est_pose_bool = False

        elif topic == "/scan_corrected" and est_pose_bool:
            previous_pcl.from_array(current_pcl.to_array())
            current_pcl.from_array(to_pcl_array(msg))
            if dt != 0:
                if dt > 1:
                    queued_pcl_list = []
                else:
                    queued_pcl_list.append(previous_pcl)
                    if len(queued_pcl_list) > 1: #queue size = 2 less than 1521*2*3 points
                        queued_pcl_list.pop(0)

                    previous_pcl_array = np.concatenate(([queue_.to_array() for queue_ in queued_pcl_list]), axis=0)
                    previous_pcl.from_array(previous_pcl_array)

            if not time:
                time.append(msg.header.stamp.to_sec())
                prev_tr = np.zeros((1, 3))
                scan_time.append(msg.header.stamp.to_sec())
                ros_time.append(t.to_sec())
                yaw = [0]
            if previous_pcl.size is not 0:
                if len(scan_time) == 1:
                    init_q = get_yaw_quaternion(imu_rot)
                    prev_q = Quaternion(1, 0, 0, 0)
                    q_array = [prev_q]
                icp = current_pcl.make_IterativeClosestPoint()
                # Final = icp.align()
                converged, transf, transf_current_pcl, fitness = icp.icp(current_pcl, previous_pcl)

                # check if the rotation matrix is valid (i.e. det==1)
                if not np.isclose(np.linalg.det(transf[0:3, 0:3]), 1.0):
                    converged = False

                if converged:

                    time.append(msg.header.stamp.to_sec())
                    dt = time[-1] - time[-2]

                    if 0 < dt <= 0.5: #if more than 10 scans (i.e. .5s) have been dropped, do save the pose (same pose as previous)
                        # get covariance matrix
                        pose_orientation, pose_translation = matrix4_to_rot_trans(imu_rot, transf, mode='2d')
                        #pose_orientation, pose_translation, cov = get_cov(transf, current_pcl, transf_current_pcl)

                        if np.linalg.norm(pose_translation)/dt*3.6 < 40: # the speed between two scans should not be greater than 40 kph
                            prev_tr = np.append(prev_tr, [prev_q.rotate(pose_translation) + prev_tr[-1]],
                                                axis=0)
                            prev_q = init_q.conjugate*pose_orientation
                            q_array.append(prev_q)

                            scan_time.append(msg.header.stamp.to_sec())
                            ros_time.append(t.to_sec())
                else:
                    dt = 0

    print '%i poses were calculated' % len(scan_time)
    print 'Interpolate the poses (orientation, position)'
    interp_time = np.linspace(scan_time[0], scan_time[-1], np.round((scan_time[-1] - scan_time[0])*f_pose))
    tr = np.zeros((interp_time.size, 3))
    interp_ros_time = np.linspace(ros_time[0], ros_time[-1], interp_time.size)
    ori = get_interpolated_quat(interp_time, scan_time, q_array)
    tr[:, 0] = np.interp(interp_time, scan_time, prev_tr[:, 0])
    tr[:, 1] = np.interp(interp_time, scan_time, prev_tr[:, 1])

    print 'Change coordinates to footprint_frame'
    # rotate from laser_frame to footprint frame
    laser_rd = -(90+9)*math.pi/180
    X = np.cos(laser_rd) * tr[:, 1] - np.sin(laser_rd) * tr[:, 0]
    Y = np.sin(laser_rd) * tr[:, 1] + np.cos(laser_rd) * tr[:, 0]
    tr[:, 1] = X
    tr[:, 0] = Y
    # rotate the orientation
    '''rot_ = Quaternion(axis=[0, 0, 1], radians=laser_rd)
    ori = [rot_*o for o in ori]'''
    print ori
    plt.plot(X, Y)
    plt.axis('equal')
    plt.show()
    print 'Writing the poses in the /scan_pose topic'
    write_pose_to_bag(bag, interp_time, interp_ros_time, ori, tr)
    print 'Re-indexing the bag'
    for done in bag.reindex():
        pass
    bag.close()
    print 'Done'


copyfile('./test2/sync_filtered_test.bag', './test2/sync_filtered_test_with_pose.bag')
bag = rosbag.Bag('./test2/sync_filtered_test_with_pose.bag', 'a')
angles = np.linspace(-95, 95, 1521)*math.pi/180.
add_pose_estimation(bag)
# bag.close()
