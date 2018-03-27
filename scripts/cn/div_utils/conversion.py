import rosbag, math
import csv
import re
import rospy
from datetime import datetime
from time import mktime

from std_msgs.msg import UInt32
from sensor_msgs.msg import Imu, MagneticField, LaserScan # Imu, Mag message from phidgets
from geometry_msgs.msg import Vector3

G = 9.81


class ToRosbag:
    log_filename = ''
    save_folder = ''
    lidar_filename = ''
    bag_name = 'test.bag'

    def __init__(self, save_folder=None, log_filename=None, lidar_filename=None, bag_name=None):

        if save_folder is not None:
            self.save_folder = save_folder
        if log_filename is not None:
            self.log_filename = log_filename
        if lidar_filename is not None:
            self.lidar_filename = lidar_filename
        if bag_name is not None:
            self.bag_name = bag_name

    def fix_csv_type(self, ln):

        line_out = dict()
        if re.match('Message', ln['TimeFromStart']) is None:
            for key in ln:
                if key:
                    if not ln[key]:
                        line_out[key] = float('nan')
                    else:
                        line_out[key] = float(ln[key])

        return line_out

    def run(self):
        # parse time from log and create rostime instance
        start_time = datetime.strptime(self.log_filename[4:18], '%Y%m%d_%H%M%S')
        ros_start_time = rospy.Time(secs=mktime(start_time.timetuple()))

        bag = rosbag.Bag(self.save_folder + self.bag_name, 'w')

        SCAN = 0
        TIME = 1
        WRITE = 2
        CONTINUE = -1

        STATE = CONTINUE

        with open(self.save_folder + self.lidar_filename) as lidarData:
            initialTimeStamp = 0
            Laserscan_msg = LaserScan()
            # generate the constant parts of the scan messages
            Laserscan_msg.header.frame_id = 'laser'
            Laserscan_msg.angle_increment = .125 * math.pi / 180
            Laserscan_msg.angle_min = -95 * math.pi / 180
            Laserscan_msg.angle_max = 95 * math.pi / 180
            Laserscan_msg.range_min = .023
            Laserscan_msg.range_max = 120.
            Laserscan_msg.scan_time = 0.05
            Laserscan_msg.time_increment = Laserscan_msg.angle_increment / (2 * math.pi) * Laserscan_msg.scan_time
            for line in lidarData:

                if STATE == SCAN:
                    scan = [float(x)/1000 for x in line.split(';')] #convert to meters and float
                    Laserscan_msg.ranges = scan
                    STATE = WRITE
                elif STATE == TIME:
                    if initialTimeStamp == 0:
                        initialTimeStamp = float(line)/1000
                        laser_time = ros_start_time
                    else:
                        laser_time = ros_start_time + rospy.Duration.from_sec(float(line)/1000 - initialTimeStamp)  # add time difference from start

                    Laserscan_msg.header.stamp = laser_time
                    STATE = CONTINUE

                if STATE == CONTINUE:
                    if re.match('\[timestamp\]', line):
                        STATE = TIME
                    elif re.match('\[scan\]', line):
                        STATE = SCAN
                elif STATE == WRITE:
                    bag.write('/scan', Laserscan_msg, t=laser_time)
                    STATE = CONTINUE

        #print('Laser duration: %.3s\n' % (laser_time-ros_start_time)/10**9)
        with open(self.save_folder + self.log_filename, 'rb') as csvData:
            excelDialect = csv.excel()
            excelDialect.skipinitialspace = True
            reader = csv.DictReader(csvData, dialect=excelDialect)
            Imu0_msg = Imu()
            Mag0_msg = MagneticField()
            Imu1_msg = Imu()
            Mag1_msg = MagneticField()

            seq = 0
            for line in reader:
                line = self.fix_csv_type(line)
                seq += 1
                if len(line) != 0:

                    line_time = ros_start_time + rospy.Duration(line['TimeFromStart'])  # add time difference from start
                    # define IMU0 in imu0_link
                    Imu0_msg.angular_velocity = Vector3(x=line['AngRateX']/180, y=line['AngRateY']/180, z=line['AngRateZ']/180)
                    Imu0_msg.linear_acceleration = Vector3(x=line['AccX']*G, y=line['AccY']*G, z=line['AccZ']*G)
                    Imu0_msg.header.stamp = line_time
                    Imu0_msg.header.frame_id = 'imu0_link'
                    # define MAG0 in imu0_link
                    Mag0_msg.magnetic_field = Vector3(x=line['MagFieldX']* 0.0001, y=line['MagFieldY']* 0.0001, z=line['MagFieldZ']* 0.0001)
                    Mag0_msg.header.stamp = line_time
                    Mag0_msg.header.frame_id = 'imu0_link'
                    # define IMU1 in imu1_link
                    Imu1_msg.angular_velocity = Vector3(x=line['AngRateX_2']/180, y=line['AngRateY_2']/180, z=line['AngRateZ_2']/180)
                    Imu1_msg.linear_acceleration = Vector3(x=line['AccX_2'] * G, y=line['AccY_2'] * G, z=line['AccZ_2'] * G)
                    Imu1_msg.header.stamp = line_time
                    Imu1_msg.header.frame_id = 'imu1_link'
                    # define MAG1 in imu1_link
                    Mag1_msg.magnetic_field = Vector3(x=line['MagFieldX_2']* 0.0001, y=line['MagFieldY_2']* 0.0001, z=line['MagFieldZ_2']* 0.0001)
                    Mag1_msg.header.stamp = line_time
                    Mag1_msg.header.frame_id = 'imu1_link'
                    # write in the bag
                    bag.write('/imu0/data_raw', Imu0_msg, t=line_time)
                    bag.write('/imu0/mag', Mag0_msg, t=line_time)
                    bag.write('/imu1/data_raw', Imu1_msg, t=line_time)
                    bag.write('/imu1/mag', Mag1_msg, t=line_time)



        print 'Done'
        bag.close()

