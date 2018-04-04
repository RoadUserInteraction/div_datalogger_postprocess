import rosbag
import rospy
import numpy as np
import roslib
import matplotlib.pyplot as plt
import seaborn
seaborn.set()
from filter.quaternion import Quaternion
from pyquaternion import Quaternion as Quat
import scipy
import scipy.fftpack
import pylab

def plot_fft(signal):
    signal = np.asarray(signal)
    FFT = abs(scipy.fft(signal))
    freqs = scipy.fftpack.fftfreq(signal.size, .01)
    plt.plot(freqs, 20 * scipy.log10(FFT))
    plt.show()

def plot_linear_acceleration(bag):
    x, y, z = [], [], []
    i = 0
    for topic, msg, t in bag.read_messages(topics="/imu0/data"):
        x.append(msg.linear_acceleration.x)
        y.append(msg.linear_acceleration.y)
        z.append(msg.linear_acceleration.z)
        if i==2000:
            print x[i], y[i], z[i]
        i += 1

    plt.plot(x, label='a_x')
    plt.plot(y, label='a_y')
    plt.plot(z, label='a_z')
    plt.legend()
    plt.show()

def plot_corrected_angular_velocity(bag):
    x, y, z = [], [], []
    x_, y_, z_ = [], [], []
    imu_2_footprint = np.array((0, -1.03, .73))
    dt = 1./250 #s
    i = 0
    prev_ang_vel = None
    velocity = np.array((0, 0, 0), dtype=np.float64)
    frame_ori = Quat(matrix=np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=np.float64))

    for topic, msg, t in bag.read_messages(topics="/imu/data", start_time=rospy.Time(bag.get_start_time())+rospy.Duration(60)):
        curr_ori = Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        curr_ang_vel = np.asarray((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
        curr_time = msg.header.stamp.to_sec()
        if prev_ang_vel is not None:
            curr_accel = curr_ori.rotate(
                np.asarray((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)))

            '''ang_accel = (curr_ori.rotate(curr_ang_vel) - prev_ori.rotate(prev_ang_vel))/dt
            accel_footprint = curr_accel \
                              + np.cross(np.cross(curr_ori.rotate(curr_ang_vel), curr_ori.rotate(imu_2_footprint)), curr_ori.rotate(curr_ang_vel)) \
                              + np.cross(curr_ori.rotate(imu_2_footprint), ang_accel)'''
            velocity += (curr_accel - np.array((0, 0, 9.8))) * (curr_time-prev_time)
            # print velocity
            x_.append(velocity[0])
            y_.append(velocity[1])
            z_.append(velocity[2])
        prev_time = curr_time
        prev_ang_vel = curr_ang_vel
        prev_ori = curr_ori

        #i += 1
    plt.plot(x_, label='footprint a_x')
    plt.plot(y_, label='footprint a_y')
    plt.plot(z_, label='footprint a_z')
    plt.legend()
    plt.show()


def plot_corrected_linear_acceleration(bag):
    x, y, z = [], [], []
    x_, y_, z_ = [], [], []
    i = 0
    for topic, msg, t in bag.read_messages(topics="/imu0/data"):
        q = Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        a_ = q.rotate((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        x.append(msg.linear_acceleration.x)
        y.append(msg.linear_acceleration.y)
        z.append(msg.linear_acceleration.z)
        x_.append(a_[0])
        y_.append(a_[1])
        z_.append(a_[2])

        if i==2000:
            print x_[i], y_[i], z_[i]
        i += 1

    plt.plot(x, label='a_x')
    plt.plot(y, label='a_y')
    plt.plot(z, label='a_z')
    plt.plot(x_, label='corr a_x')
    plt.plot(y_, label='corr a_y')
    plt.plot(z_, label='corr a_z')
    plt.legend()
    plt.show()

def plot_orientation(bag):
    yaw, pitch, roll = [], [], []
    scalar = []
    x, y, z = [], [], []
    i = 0
    for topic, msg, t in bag.read_messages(topics="/imu0/data"):
        q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        euler = q.to_euler123()
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])
        scalar.append(msg.orientation.w)
        x.append(msg.orientation.x)
        y.append(msg.orientation.y)
        z.append(msg.orientation.z)
        if i==2000:
            print msg.orientation
        i += 1

    plt.plot(yaw, label='yaw')
    plt.plot(pitch, label='pitch')
    plt.plot(roll, label='roll')
    plt.legend()
    plt.show()

    plt.plot(scalar, label='scalar')
    plt.plot(x, label='x')
    plt.plot(y, label='y')
    plt.plot(z, label='z')
    plt.legend()
    plt.show()


bag = rosbag.Bag("./test3/2018-03-21-12-48-27_0.bag")
plot_corrected_angular_velocity(bag)
bag.close()
