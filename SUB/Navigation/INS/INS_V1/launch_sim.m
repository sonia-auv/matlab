clear all;
clc;
close all;

path('../lib',path);

choice = 3;

if choice == 1
    file_name = 'TEST_PISCINE_20160123';
elseif choice == 2
    file_name = 'TEST_PISCINE_20160123_HEADING';
elseif choice == 3
    file_name = 'TEST_PISCINE_20160305_SQUARE';
end

load(strcat(file_name,'/DATA_IMU'));
load(strcat(file_name,'/DATA_MAG'));
load(strcat(file_name,'/DATA_DVL'));
load(strcat(file_name,'/DATA_BARO'));

run('../lib/init_INS_V1');

save Sensors_IMU Sensors_IMU
save Sensors_MAG Sensors_MAG
save Sensors_DVL Sensors_DVL
save Sensors_BARO Sensors_BARO