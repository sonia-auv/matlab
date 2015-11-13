clear all;
clc;
close all;

path('../lib',path);

choice = 1;

if choice == 1
    load('data_input/TEST_PISCINE_20151017/DATA_IMU');
    load('data_input/TEST_PISCINE_20151017/DATA_DVL');
    load('data_input/TEST_PISCINE_20151017/DATA_BARO');
    save DATA_IMU DATA_IMU
    save DATA_DVL DATA_DVL
    save DATA_BARO DATA_BARO
end

run('../lib/init_constants');
%run('../lib/initialization');