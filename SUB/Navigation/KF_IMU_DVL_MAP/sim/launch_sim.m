clear all;
clc;
close all;

path('../lib',path);

choice = 3;

if choice == 1
    load('data_input/TEST_PISCINE_20151017_212743/DATA_IMU');
    load('data_input/TEST_PISCINE_20151017_212743/DATA_DVL');
    load('data_input/TEST_PISCINE_20151017_212743/DATA_BARO');
    save DATA_IMU DATA_IMU
    save DATA_DVL DATA_DVL
    save DATA_BARO DATA_BARO
elseif choice == 2
    load('data_input/TEST_PISCINE_20151017_213236/DATA_IMU');
    load('data_input/TEST_PISCINE_20151017_213236/DATA_DVL');
    load('data_input/TEST_PISCINE_20151017_213236/DATA_BARO');
    save DATA_IMU DATA_IMU
    save DATA_DVL DATA_DVL
    save DATA_BARO DATA_BARO
elseif choice == 3
    load('data_input/TEST_PISCINE_20151114/DATA_IMU');
    save DATA_IMU DATA_IMU
end

run('../lib/init_constants');
%run('../lib/initialization');