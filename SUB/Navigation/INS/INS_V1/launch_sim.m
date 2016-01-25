clear all;
clc;
close all;

path('../lib',path);

choice = 1;

if choice == 1
    load('TEST_PISCINE_20160123/DATA_IMU');
    load('TEST_PISCINE_20160123/DATA_MAG');
    load('TEST_PISCINE_20160123/DATA_DVL');
elseif choice == 2
    load('TEST_PISCINE_20160123_HEADING/DATA_IMU');
    load('TEST_PISCINE_20160123_HEADING/DATA_MAG');
    load('TEST_PISCINE_20160123_HEADING/DATA_DVL');
end

run('../lib/init_INS_V1');
