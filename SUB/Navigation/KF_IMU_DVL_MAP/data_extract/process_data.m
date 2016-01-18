clear all;
clc;

path('data_input/',path);
path('data_output/',path);
path('lib/',path);

file_imu = '_slash_imu_slash_data';
file_dvl = '_slash_provider_dvl_PD0Packet';

disp('Extracting data...');

process_imu(file_imu);
process_dvl(file_dvl);

disp('Data successfully extracted.');
