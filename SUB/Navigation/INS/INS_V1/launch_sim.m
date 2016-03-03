clear all;
clc;
close all;

path('../lib',path);

choice = 1;

if choice == 1
    file_name = 'TEST_PISCINE_20160123';
elseif choice == 2
    file_name = 'TEST_PISCINE_20160123_HEADING';
end

load(strcat(file_name,'/DATA_IMU'));
load(strcat(file_name,'/DATA_MAG'));
load(strcat(file_name,'/DATA_DVL'));

run('../lib/init_INS_V1');

save Sensors_IMU Sensors_IMU
save Sensors_MAG Sensors_MAG
save Sensors_BARO Sensors_BARO
save Sensors_DVL Sensors_DVL