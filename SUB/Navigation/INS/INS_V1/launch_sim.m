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
elseif choice == 4
    file_name = 'TEST_PISCINE_20160305_CIRCLE';
elseif choice == 5
    file_name = 'TEST_PISCINE_20160305_CIRCLE_HEADING';
elseif choice == 6
    file_name = 'TEST_PISCINE_20160305_OUTOFWATER';
end

load(strcat(file_name,'/DATA_IMU'));
load(strcat(file_name,'/DATA_MAG'));
load(strcat(file_name,'/DATA_DVL'));
load(strcat(file_name,'/DATA_BARO'));
if choice == 3
    load(strcat(file_name,'/DATA_ODOM'));
    for j = 1:(size(DATA_IMU,2))
        DATA_ODOM(1,j) = DATA_IMU(1,j);
    end
else
    for i = 1:11
        for j = 1:(size(DATA_IMU,2))
            if i == 1
                DATA_ODOM(i,j) = DATA_IMU(i,j);
            else
                DATA_ODOM(i,j) = 0;
            end
        end
    end
end

run('../lib/init_INS_V1');

save Sensors_IMU Sensors_IMU
save Sensors_MAG Sensors_MAG
save Sensors_DVL Sensors_DVL
save Sensors_BARO Sensors_BARO