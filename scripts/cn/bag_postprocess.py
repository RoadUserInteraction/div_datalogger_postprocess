from div_utils import sync
from rosbag.rosbag_main import filter_cmd

save_folder = './test2/'
lidar_filename = '2015_04_17_08_45_34_490.ubh'
log_filename = 'Log_20150417_085106.csv'
bag_name = 'test.bag'
filtered_bag_name = 'filtered_test.bag'
sync_filtered_bag_name = 'sync_filtered_test.bag'
trunc_bag_name = 'trunc_test.bag'
# convert CSV and UBH to rosbag, lidar and data might be unsynchronized
'''convert = conversion.ToRosbag(lidar_filename=lidar_filename,
                              log_filename=log_filename,
                              save_folder=save_folder,
                              bag_name=bag_name)
# do the actual conversion
convert.run()

# filter the IMUs orientation
imufilter = imu_filter.imuFilter(bagfile_path=save_folder+bag_name,
                                 output_bagfile_path=save_folder+filtered_bag_name,
                                 beta0=0.1, beta1=0.1,
                                 gyro_zeroing_init=10, gyro_zeroing_duration=120, zero_gyro=True)

# apply the madgwick filter
imufilter.run()

# synchronize lidar data with imu, using the steering events from the imu1
lidarsync = sync.Sync(bagfile_path=save_folder + filtered_bag_name,
                      output_bagfile_path=save_folder+sync_filtered_bag_name,
                      low_time_limit=70,
                      jitter_coef=.949,
                      f=100.,
                      angle_threshold=-.03,
                      scan_threshold=0.03,
                      opts_plot=True)

# do the synchronization
lidarsync.run()
lidarsync.close()'''

