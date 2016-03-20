
clear;
clc;

data_imu = 1;
data_mag = 1;
data_dvl = 1;
data_baro = 1;
data_odom = -1;

file_name = 'test_20160319/2016-03-19-21-10-49.bag';
file_name_odom = 'odometry/2016-03-15-19-56-05.bag';

if data_imu == 1 || data_mag == 1 || data_dvl == 1 || data_baro == 1
    bag = rosbag(file_name);
end

if data_imu == 1
    bagIMU = select(bag, 'Topic', 'provider_imu/imu');
    disp('Extracting IMU data...');
    [tsIMU, colsIMU] = timeseries(bagIMU);
    DATA_IMU(1,:) = tsIMU.Time';
    DATA_IMU(2:4,:) = tsIMU.Data(:,11:13)';
    DATA_IMU(5:7,:) = tsIMU.Data(:,8:10)';
    save DATA_IMU DATA_IMU
end
if data_mag == 1
    bagMAG = select(bag, 'Topic', 'provider_imu/magnetic_field');
    disp('Extracting MAG data...');
    [tsMAG, colsMAG] = timeseries(bagMAG);
    DATA_MAG(1,:) = tsMAG.Time';
    DATA_MAG(2:4,:) = tsMAG.Data(:,4:6)';
    save DATA_MAG DATA_MAG
end
if data_dvl == 1
    bagDVL = select(bag, 'Topic', 'provider_dvl/twist');
    disp('Extracting DVL data...');
    [tsDVL, colsDVL] = timeseries(bagDVL);
    DATA_DVL(1,:) = tsDVL.Time';
    DATA_DVL(2:4,:) = tsDVL.Data(:,4:6)';
    save DATA_DVL DATA_DVL
end
if data_baro == 1
    bagBARO = select(bag, 'Topic', '/provider_can/barometer_fluidpress_msgs');
    disp('Extracting BARO data...');
    [tsBARO, colsBARO] = timeseries(bagBARO);
    DATA_BARO(1,:) = tsBARO.Time';
    DATA_BARO(2,:) = tsBARO.Data(:,1)';
    save DATA_BARO DATA_BARO
end
if data_odom == 1
    bag_odom = rosbag(file_name_odom);
    bagODOM = select(bag_odom, 'Topic', 'proc_navigation/odom');
    disp('Extracting ODOM data...');
    [tsODOM, colsODOM] = timeseries(bagODOM);
    DATA_ODOM(1,:) = tsODOM.Time';
    DATA_ODOM(2:4,:) = tsODOM.Data(:,4:6)';
    DATA_ODOM(5:7,:) = tsODOM.Data(:,11:13)';
    DATA_ODOM(8:11,:) = tsODOM.Data(:,7:10)';
    save DATA_ODOM DATA_ODOM
end
disp('Data succesfully extracted...');
