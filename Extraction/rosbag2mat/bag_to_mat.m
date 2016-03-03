clear;
clc;

file_name = 'test_20160123/2016-01-23-22-29-10.bag';

bag = rosbag(file_name);
bagIMU = select(bag, 'Topic', 'provider_imu/imu');
bagMAG = select(bag, 'Topic', 'provider_imu/magnetic_field');

 [tsIMU, colsIMU] = timeseries(bagIMU);
 [tsMAG, colsMAG] = timeseries(bagMAG);

DATA_IMU(1,:) = tsIMU.Time';
DATA_IMU(2:4,:) = tsIMU.Data(:,11:13)';
DATA_IMU(5:7,:) = tsIMU.Data(:,8:10)';
DATA_MAG(1,:) = tsMAG.Time';
DATA_MAG(2:4,:) = tsMAG.Data(:,4:6)';

save DATA_IMU DATA_IMU
save DATA_MAG DATA_MAG

%%
clear;
clc;

file_name = 'test_20160123/2016-01-23-22-29-08.bag';

bag = rosbag(file_name);
bagDVL = select(bag, 'Topic', 'provider_dvl/twist');

 [tsDVL, colsDVL] = timeseries(bagDVL);


DATA_DVL(1,:) = tsDVL.Time';
DATA_DVL(2:4,:) = tsDVL.Data(:,4:6)';

save DATA_DVL DATA_DVL
