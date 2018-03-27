import rosbag
import rospy
from filter.quaternion import Quaternion
import numpy as np
import math
import scipy
from matplotlib import pyplot as plt
import seaborn

seaborn.set()

class Sync:

    # define default variables
    low_time_limit = 70  # seconds
    jitter_coef = .949
    f = 100  # Hz, the update frequency of the IMU
    opts_plot = False
    bagfile_path = 'in.bag'
    output_bagfile_path = 'out.bag'
    scan_threshold = .05
    angle_threshold = -.05

    def __init__(self, bagfile_path=None, output_bagfile_path=None, low_time_limit=None, jitter_coef=None, f=None,
                 scan_threshold=None, angle_threshold=None, opts_plot=None):
        if bagfile_path is not None:
            self.bagfile_path=bagfile_path
            self.bag = rosbag.Bag(bagfile_path)
        if output_bagfile_path is not None:
            self.output_bagfile_path=output_bagfile_path
            self.output_bag = rosbag.Bag(output_bagfile_path, 'w')
        if low_time_limit is not None:
            self.low_time_limit = low_time_limit
        if jitter_coef is not None:
            self.jitter_coef = jitter_coef
        if f is not None:
            self.f = float(f)
        if scan_threshold is not None:
            self.scan_threshold = scan_threshold
        if angle_threshold is not None:
            self.angle_threshold = angle_threshold
        if opts_plot is not None:
            self.opts_plot = opts_plot

    def fixangle(self, x):
        if x > 0:
            x -= math.pi
        else:
            x += math.pi

        return x

    def my_range(self, start, end, step):
        while start <= end:
            yield start
            start += step

    def get_delay(self, a, b):

        corr = scipy.correlate(a, b, mode='same')
        return (np.argmax(abs(corr))-a.size/2)/self.f

    def get_sync_coef(self):

        bag = self.bag
        imu_time = []
        init_time = []
        anglez = []
        scan_time = []
        angles = np.zeros([0, 0])
        counts = []
        for topic, msg, t in bag.read_messages(topics=["/imu1/data", "/scan"]):

            if not init_time:
                init_time = msg.header.stamp.to_sec()

            if topic == "/imu1/data":
                # extract imu time and euler[2] of imu1 that corresponds to the steering angle
                imu_time.append(msg.header.stamp.to_sec() - init_time)
                q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
                euler = q.to_euler_angles()
                anglez.append(euler[2])

            else:
                '''extract the number of steps that touches an object within the 120m, limit the angle range search within 
                -80, +80. when counts = 0, it is probable that the bike frame is inclined towards the road (leftwards), 
                when counts == max(steps), it means that most probably the bike is inclined to the right'''
                if angles.size == 0:
                    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
                    indToUse = np.abs(angles) * 180 / math.pi < 80

                scan_time.append(msg.header.stamp.to_sec() - init_time)
                ranges = np.asarray(msg.ranges)
                counts.append(np.sum(ranges[indToUse] == 120))

        scan_angle = np.asarray(counts)
        imu_time = np.asarray(imu_time)
        imu_time -= imu_time[0]
        scan_time = (np.asarray(scan_time)-imu_time[0]) * self.jitter_coef  # shrink scan_time if jitter_coef < 1
        # generate a 0-1 array, 1 corresponds to when the bike steered and that most of the laser beam touches an object within 120 m
        limitScan = np.logical_and(scan_angle / float(np.sum(indToUse)) < self.scan_threshold, scan_time > self.low_time_limit)
        # because the angle is z, the angle oscillates around pi -pi, need to recenter the angle
        signal = -np.asarray([self.fixangle(x) for x in anglez])
        # generate a 0-1 array, 1 corresponds to when the bike steered (euler[2] of imu1)
        limitImu = np.logical_and(signal < self.angle_threshold, imu_time > self.low_time_limit)
        # upsample the limitScan to match the time vector of the imu_time
        limitScan_resampled = np.interp(imu_time, scan_time, limitScan)

        # get delay
        t_delay = self.get_delay(limitImu, limitScan_resampled)

        if self.opts_plot:

            # plot the graphs
            fig, ax = plt.subplots()
            #plt.plot(imu_time, signal, label='steering angle')
            plt.plot(imu_time, limitImu, label='threshold steering angle')
            #plt.plot(imu_time, limitScan_resampled, label='original scan inclination')
            #plt.plot(scan_time, scan_angle/ float(np.sum(indToUse)), label='scan angle')
            plt.plot(scan_time + t_delay, limitScan, label='inclination from scan, corrected')
            ax.legend()
            plt.show()

        return t_delay

    def run(self):
        t_delay = self.get_sync_coef()
        print 'The delay has been estimated at %.3f s' % t_delay
        bag = self.bag
        output_bag = self.output_bag
        #get reference time from imu for sync
        for topic, msg, t in bag.read_messages(topics='/imu1/data'):
            init_time = msg.header.stamp.to_sec()
            break

        for topic, msg, t in bag.read_messages():
            if topic == '/scan':
                output_bag.write('/scan', msg, t=msg.header.stamp)
                previous = msg.header.stamp.to_sec()
                msg.header.stamp = rospy.Time.from_sec((msg.header.stamp.to_sec() - init_time)* self.jitter_coef + t_delay + init_time)
                output_bag.write('/scan_corrected', msg, t=msg.header.stamp)
            else:
                output_bag.write(topic, msg, t=t)

        print 'Sync done'

    def close(self):
        self.bag.close()
        self.output_bag.close()
