clear;

file_name = 'data_test_imu_dvl.bag';

bag = rosbag(file_name);
bagIMU = select(bag, 'MessageType', 'sensor_msgs/Imu')
bagDVL = select(bag, 'MessageType', 'provider_dvl/PD0Packet')


% msgsIMU = readMessages(bagIMU)
% msgsDVL = readMessages(bagDVL)

 [tsIMU, colsIMU] = timeseries(bagIMU)
% [tsDVL, colsDVL] = timeseries(bagDVL)


