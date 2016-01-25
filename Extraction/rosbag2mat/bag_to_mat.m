clear;
clc;

file_name = 'test_20160123/2016-01-23-22-31-08.bag';

bag = rosbag(file_name);
bagIMU = select(bag, 'Topic', 'provider_imu/imu');
bagMAG = select(bag, 'Topic', 'provider_imu/magnetic_field');
%bagDVL = select(bag, 'Topic', 'provider_dvl/twist');

 [tsIMU, colsIMU] = timeseries(bagIMU);
 [tsMAG, colsMAG] = timeseries(bagMAG);
% [tsDVL, colsDVL] = timeseries(bagDVL);

DATA_IMU(1,:) = tsIMU.Time';
DATA_IMU(2:4,:) = tsIMU.Data(:,11:13)';
DATA_MAG(1,:) = tsMAG.Time';
DATA_MAG(2:4,:) = tsMAG.Data(:,4:6)';
%DATA_DVL(1,:) = tsDVL.Time';
%DATA_DVL(2:4,:) = tsDVL.Data(:,4:6)';
save DATA_IMU DATA_IMU
save DATA_MAG DATA_MAG
%save DATA_DVL DATA_DVL