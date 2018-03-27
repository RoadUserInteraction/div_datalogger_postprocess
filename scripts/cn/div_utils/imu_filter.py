#!/usr/bin/env python
import rosbag
import rospy
import operator
from filter.madgwickahrs import *
from compass import compass
import math
import numpy as np
from geometry_msgs.msg import Quaternion


class imuFilter:
    beta0 = .1
    beta1 = .1
    bagfile_path = '/home/christian-nils/CN/Program/python/test.bag'  # rospy.get_param('bagfile')
    output_bagfile_path = '/home/christian-nils/CN/Program/python/test_corrected.bag'
    gyro_zeroing_init = 0  # s
    gyro_zeroing_duration = 10  # s
    zero_gyro = False

    def __init__(self, beta0=None, beta1=None, bagfile_path=None, output_bagfile_path=None, gyro_zeroing_init=None,
                 gyro_zeroing_duration=None,zero_gyro=None):
        if beta0 is not None:
            self.beta0 = float(beta0)
        if beta1 is not None:
            self.beta1 = float(beta1)
        if bagfile_path is not None:
            self.bagfile_path = bagfile_path
        if output_bagfile_path is not None:
            self.output_bagfile_path = output_bagfile_path
        if gyro_zeroing_duration is not None:
            self.gyro_zeroing_duration = gyro_zeroing_duration
        if gyro_zeroing_init is not None:
            self.gyro_zeroing_init = gyro_zeroing_init
        if zero_gyro is not None:
            self.zero_gyro = zero_gyro

    def vector3tolist(self, vec3):
        return [vec3.x, vec3.y, vec3.z]

    def fixangle(self, x):
        if x > 0:
            x -= math.pi
        else:
            x += math.pi

        return x

    def run(self):

        orientation0 = MadgwickAHRS(beta=self.beta0)  # do not use slash divide, the behaviour is different from Python3
        orientation1 = MadgwickAHRS(beta=self.beta1)  # do not use slash divide, the behaviour is different from Python3

        bag = rosbag.Bag(self.bagfile_path)

        out_bag = rosbag.Bag(self.output_bagfile_path, 'w')
        new0Mag, new1Mag = False, False
        new0Imu, new1Imu = False, False
        x0, x1 = [], []
        y0, y1 = [], []
        z0, z1 = [], []
        time = []
        init_time = None

        if self.zero_gyro:
            standing_still_time = self.gyro_zeroing_init
            zeroing_period = self.gyro_zeroing_duration
            # mag correction parameters to be changed
            magObj = compass(cc=[0.51075, 0.18820, -0.07456, -0.02209, 1.87163, 1.87640, 2.12565, -0.04000, -0.04084, -0.03552, 0.09073, -0.04258, 0.11056])
            for topic, msg, t in bag.read_messages(topics=['/imu0/data_raw', '/imu1/data_raw']):
                if not init_time:
                    init_time = t.to_sec()
                    time.append(0)
                else:
                    time.append(t.to_sec() - init_time)

                if standing_still_time <= time[-1] < standing_still_time+zeroing_period:
                    if topic == '/imu0/data_raw':
                        x0.append(msg.angular_velocity.x)
                        y0.append(msg.angular_velocity.y)
                        z0.append(msg.angular_velocity.z)
                    else:
                        x1.append(msg.angular_velocity.x)
                        y1.append(msg.angular_velocity.y)
                        z1.append(msg.angular_velocity.z)
                elif time[-1] >= standing_still_time+zeroing_period:
                    break

            drift_offset0 = [np.mean(x0), np.mean(y0), np.mean(z0)]
            drift_offset1 = [np.mean(x1), np.mean(y1), np.mean(z1)]
        else:
            drift_offset0 = [0, 0, 0]
            drift_offset1 = [0, 0, 0]

        init_time = None
        time = []
        for topic, msg, t in bag.read_messages():
            # treat only the messages that are within the recording time period
            if bag.get_start_time() < msg.header.stamp.to_sec() < bag.get_end_time():
                if not init_time:
                    init_time = msg.header.stamp.to_sec()

                if topic == '/imu0/mag':
                    new0Mag = True
                    mag0Msg = msg
                elif topic == '/imu0/data_raw':
                    new0Imu = True
                    imu0Msg = msg
                elif topic == '/imu1/mag':
                    new1Mag = True
                    mag1Msg = msg
                elif topic == '/imu1/data_raw':
                    new1Imu = True
                    imu1Msg = msg
                out_bag.write(topic, msg, t=t)

                if new0Mag & new0Imu:

                    orientation0.update(map(operator.sub, self.vector3tolist(imu0Msg.angular_velocity), drift_offset0),
                                        self.vector3tolist(imu0Msg.linear_acceleration),
                                        magObj.correct(self.vector3tolist(mag0Msg.magnetic_field)),
                                        imu0Msg.header.stamp.to_sec())
                    new0Mag = False
                    new0Imu = False
                    imu_corrected_msg = imu0Msg
                    imu_corrected_msg.orientation = Quaternion(orientation0.quaternion[1], orientation0.quaternion[2],
                                                               orientation0.quaternion[3], orientation0.quaternion[0])
                    out_bag.write('/imu0/data', imu_corrected_msg, t=t)
                    time.append(imu0Msg.header.stamp.to_sec()-init_time)

                elif new1Mag & new1Imu:
                    orientation1.update(map(operator.sub, self.vector3tolist(imu1Msg.angular_velocity), drift_offset1),
                                        self.vector3tolist(imu1Msg.linear_acceleration),
                                        magObj.correct(self.vector3tolist(mag1Msg.magnetic_field)),
                                        imu1Msg.header.stamp.to_sec())
                    new1Mag = False
                    new1Imu = False
                    imu_corrected_msg = imu1Msg
                    imu_corrected_msg.orientation = Quaternion(orientation1.quaternion[1], orientation1.quaternion[2],
                                                               orientation1.quaternion[3], orientation1.quaternion[0])
                    out_bag.write('/imu1/data', imu_corrected_msg, t=t)

        print 'Re-indexing bag...'
        #out_bag.reindex()
        out_bag.close()
        bag.close()
        print 'Done'
