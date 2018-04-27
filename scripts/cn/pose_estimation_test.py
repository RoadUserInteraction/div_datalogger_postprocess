import rosbag
import rospy
import numpy as np
from numpy import ma
import roslib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from pykalman import KalmanFilter
import math
import pcl
from shutil import copyfile
from geometry_msgs.msg import Quaternion as Quat
from geometry_msgs.msg import Point as Pt
from geometry_msgs.msg import PoseWithCovarianceStamped


def get_yaw_quaternion(imu_rot):
    return Quaternion(axis=[0, 0, 1], radians=get_yaw_rd(imu_rot))


def get_yaw_rd(imu_rot):
    v = [1, 0, 0]
    v_ = imu_rot.rotate(v)  # get the magnitude of orientation
    w = np.cross(v, v_)  # get the sign of orientation
    yaw = np.sign(-w[2]) * np.arccos(v_[0])  #
    return yaw


def matrix4_to_rot_trans(transf, mode='3d'):
    tr = transf[0:3, 3] # translation vector (x, y, z), z should be zero
    q = Quaternion(matrix=transf)  # quaternion from matrix

    if mode == '2d':
        q = get_yaw_quaternion(q)
        tr[2] = 0

    return q, tr


def to_pcl_array(scan_msg, laser_ori, imu_rot):
    pcl_array = np.empty((1521, 3), dtype=np.float32)
    ranges = np.asarray(scan_msg.ranges)
    ind_to_keep = ranges != 120

    X = ranges * np.cos(angles)
    Y = ranges * np.sin(angles)

    q = get_yaw_quaternion(imu_rot).rotate(laser_ori)
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    pcl_array[:, 0] = X*(qx**2+qw**2-qy**2- qz**2) + Y*(2*qx*qy- 2*qw*qz)
    pcl_array[:, 1] = X*(2*qw*qz + 2*qx*qy) + Y*(qw**2 - qx**2 + qy**2 - qz**2)
    pcl_array[:, 2] = 0

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


def get_measurements(bag, start_time=None, end_time=None):
    '''Initialization of variables'''
    if start_time is not None:
        start_time = rospy.Time(bag.get_start_time()) + rospy.Duration(start_time)
    if end_time is not None:
        end_time = rospy.Time(bag.get_start_time()) + rospy.Duration(end_time)
    imu_frame = Quaternion(-0.0198769629216, 0.974536168168, -0.218396560265, -0.0467665023439)
    v_z = (0, 0, 1)
    initialized = None
    new_estimated_pose = False
    est_pose_bool = False
    previous_pcl = pcl.PointCloud()
    current_pcl = pcl.PointCloud()
    measurements = []
    time = []
    scan_time = []
    imu_time = []
    ros_time = []
    q_array = []
    laser_ori = Quaternion(axis=(0,0,1), radians= 9 * math.pi / 180).rotate(Quaternion(axis=(1,0,0), radians=math.pi))
    # rotation quaternion to transform the laser_frame to the bike_frame

    for topic, msg, t in bag.read_messages(topics=['/imu/data', '/scan'],
                                           start_time=start_time,
                                           end_time=end_time):
        '''
        topics
            /imu0/data: imu attached to the bike frame, x left, y forward, z downwards
            /scan_corrected: synchronized laserscan data
        start_time, end_time:
            the time range in which the messages are read
        '''

        if topic == '/imu/data':
            # create a quaternion object with the orientation from the IMU
            curr_ori = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            # transform the acceleration from imu_frame to world_frame
            curr_accel = curr_ori.rotate(
                np.asarray((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)))



            acc_x = curr_accel[0]
            acc_y = curr_accel[1]
            ros_time.append(t)
            imu_time.append(msg.header.stamp.to_sec())
            q_array.append(get_yaw_quaternion(curr_ori))
            '''acc_x_.append(acc_x)
            acc_y_.append(acc_y)'''
            if new_estimated_pose:
                measurements.append(np.array([prev_speed[-1, 0], prev_speed[-1, 1], acc_x, acc_y]))
                new_estimated_pose = False
            else:
                measurements.append(ma.array([0, 0, acc_x, acc_y], mask=[1, 1, 0, 0]))

            # Calculate the angle between the horizontal plane and the laserscan plane
            #v_z_ = curr_ori.rotate(imu_frame.rotate(v_z))   # ERROR
            v_z_ = curr_ori * imu_frame * Quaternion(0, 0, 0, 1) * imu_frame.conjugate * curr_ori.conjugate

            # estimate pose from scan matching only if the angle is smaller than .05 rad
            if np.arccos(v_z_[3]) > -.05:
                est_pose_bool = True
            else:
                est_pose_bool = False

        elif topic == "/scan" and est_pose_bool:
            # populate the previous pointcloud
            previous_pcl.from_array(current_pcl.to_array())
            # populate the current PCL after having rectify the laser orientation and the current imu rotation w.r.t the world frame
            current_pcl.from_array(to_pcl_array(msg, laser_ori, curr_ori))
            #plot_pcl(previous_pcl, current_pcl) # you can plot here the two current pcl using this function

            # if not initialized, initialize the variables
            if initialized is None:
                time.append(msg.header.stamp.to_sec())
                prev_q = curr_ori  # orientation
                prev_speed = np.zeros((1, 3))  # set the initial speed to 0
                scan_time.append(msg.header.stamp.to_sec())
                initialized = True
            # if previous_pcl.size is 0 it means that we are at the first iteration, skip the first
            if previous_pcl.size is not 0:
                # create an ICP object
                icp = current_pcl.make_IterativeClosestPoint()
                # run the ICP algorithm
                converged, transf, transf_current_pcl, fitness = icp.icp(current_pcl, previous_pcl)

                # check if the rotation matrix is valid (i.e. det==1)
                if not np.isclose(np.linalg.det(transf[0:3, 0:3]), 1.0):
                    converged = False

                # if converged (if a match has been estimated)
                if converged:
                    time.append(msg.header.stamp.to_sec())

                    dt = time[-1] - time[-2]
                    # get the orientation and the translation of the transform
                    pose_orientation, pose_translation = matrix4_to_rot_trans(transf, mode='2d')
                    # evaluate the speed
                    new_speed = pose_translation / dt  # in bike frame
                    # if the estimated speed is higher than 40 kph, do not save the speed FIXME: this could be improved
                    if np.linalg.norm(new_speed)*3.6 < 10:
                        # use the previous pose orientation to set the speed vector into the world_frame
                        prev_speed = np.append(prev_speed, [prev_q.rotate(new_speed)], axis=0)
                        # save the pose orientation for the next iteration
                        prev_q = pose_orientation
                        new_estimated_pose = True

    return measurements, q_array, imu_time, ros_time


def get_yaw_2d(vector):
    return np.arctan2(vector[1], vector[0]) - np.arctan2(0, 1)


def write_pose_to_bag(bag, time, ros_time, tr):
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.pose.covariance = [0] * 36
    pose_msg.header.frame_id = 'base_footprint'
    for i in range(0, len(time)-1):
        pose_msg.header.stamp = rospy.Time(time[i])
        pose_msg.pose.pose.position = Pt(tr[i, 0], tr[i, 1], 0)
        # calculate orientation from the pose
        ori_q = Quaternion(axis=(0, 0, 1), radians=get_yaw_2d(np.diff(tr[i:i+2, 0:2], axis=0)[0]))
        pose_msg.pose.pose.orientation = Quat(ori_q[1], ori_q[2], ori_q[3], ori_q[0])
        bag.write('/scan_pose', pose_msg, t=ros_time[i])


def kalman(measurements):

    '''Define the different matrices to input to the Kalman filter'''
    t = 1./100 #sample time
    A = np.array([[1, 0, t, 0, 0.5 * (t ** 2), 0],
                  [0, 1, 0, t, 0, 0.5 * (t ** 2)],
                  [0, 0, 1, 0, t, 0],
                  [0, 0, 0, 1, 0, t],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]]) #transition matrix (used for the prediction)
    Q = np.array([
        [1.e-1, 0, 0, 0, 0, 0],
        [0, 1.e-1, 0, 0, 0, 0],
        [0, 0, 1.e-1, 0, 0, 0],
        [0, 0, 0, 1.e-1, 0, 0],
        [0, 0, 0, 0, 1.e-1, 0],
        [0, 0, 0, 0, 0, 1.e-1]
    ]) # transition covariance matrix
    C = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]]) #observation matrix (links the state vector to the observation/measurement vector)
    R = np.array([
        [1.e5, 0, 0, 0],
        [0, 1.e5, 0, 0],
        [0, 0, 1.e0, 0],
        [0, 0, 0, 1.e0]
    ])  # observation covariance matrix
    klm = KalmanFilter(transition_matrices=A,
                       transition_covariance=Q,
                       observation_matrices=C,
                       observation_covariance=R,
                       initial_state_mean=[0, 0, 0, 0, 0, 0])

    (smoothed_state_means, smoothed_state_covariances) = klm.smooth(measurements)

    # uncomment this block if you want to plot the kalman filter results
    plt.subplot(131)
    plt.plot(smoothed_state_means[:, 2], 'b')
    plt.plot(smoothed_state_means[:, 3], 'r')
    plt.plot([x[0] for x in measurements], 'b', marker="o")
    plt.plot([x[1] for x in measurements], 'r', marker="o")
    plt.plot((smoothed_state_means[:, 2]**2 + smoothed_state_means[:, 3]**2)**(1./2), 'k', linewidth=2)
    plt.subplot(132)
    plt.plot(smoothed_state_means[:, 0], 'b')
    plt.plot(smoothed_state_means[:, 1], 'r')
    plt.subplot(133)
    plt.plot(smoothed_state_means[:, 0], smoothed_state_means[:, 1])
    plt.show()
    return smoothed_state_means

# change the original bag
bag = rosbag.Bag("/home/chalmers/Desktop/2018-PedestrianOvertaking/SecondCollection/2018-03-21-14-13-32_1.bag")
# angles for the Hokuyo laser
angles = np.linspace(-95, 95, 1521)*math.pi/180.
print "Appending the measurement list"
# create the measurement list that is given to the kalman filter
measurements, ori_q, imu_time, ros_time = get_measurements(bag, start_time=0, end_time=60)
print "Done!"
print "Applying Kalman filter"
# Apply the filter
pose = kalman(measurements)
print "Done!"
print "Write pose to bag file"
# Copy the initial bag and open it (change the name)
'''copyfile('/home/chalmers/Desktop/2018-PedestrianOvertaking/SecondCollection/2018-03-21-14-13-32_1.bag', '/home/chalmers/Desktop/2018-PedestrianOvertaking/SecondCollection/2018-03-21-14-13-32_1_with_pose_startend.bag')
out_bag = rosbag.Bag('/home/chalmers/Desktop/2018-PedestrianOvertaking/SecondCollection/2018-03-21-14-13-32_1_with_pose_startend.bag', 'a')
# write the results from the kalman filter
write_pose_to_bag(out_bag, imu_time, ros_time, pose)
print 'Re-indexing the bag'
for done in out_bag.reindex():
    pass
out_bag.close()'''
bag.close()
print 'Done'
