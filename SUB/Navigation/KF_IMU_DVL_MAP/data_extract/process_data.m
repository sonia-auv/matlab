clear all;
clc;

path('data_input/',path);
path('data_output/',path);

file_name = 'Gate-21h27m43s';

disp('Extracting data...');

Nb_wrong = 0; % Wrong data
Nb_end = 0;

ds = datastore(strcat(file_name,'.csv'),'Delimiter',',');
Variables = ds.VariableNames;
ds.SelectedVariableNames = {'Timestamp',...
                            'ImuRec_ImuRollAngularRate', 'ImuRec_ImuPitchAngularRate', 'ImuRec_ImuYawAngularRate',...
                            'ImuRec_ImuRoll', 'ImuRec_ImuPitch', 'ImuRec_ImuYaw',...
                            'DvlRec_VelocityX', 'DvlRec_VelocityY', 'DvlRec_VelocityZ',...
                            'DvlCommunicationRec_BottomStatus', 'DvlCommunicationRec_VelocityErrorBottom'};

data = readall(ds);
format long
start_time = data.Timestamp(1);

raw_time = (data.Timestamp-start_time)/1000;

s_data = size(data.Timestamp, 1);

% Add times to matrix
raw_imu(:,1) = raw_time;
raw_dvl(:,1) = raw_time;
raw_baro(:,1) = raw_time;

% IMU
raw_imu(:,2) = zeros(s_data,1);
raw_imu(:,3) = zeros(s_data,1);
raw_imu(:,4) = zeros(s_data,1);
raw_imu(:,5) = data.ImuRec_ImuRollAngularRate/180*pi;
raw_imu(:,6) = data.ImuRec_ImuPitchAngularRate/180*pi;
raw_imu(:,7) = data.ImuRec_ImuYawAngularRate/180*pi;
raw_imu(:,8) = data.ImuRec_ImuRoll/180*pi;
raw_imu(:,9) = data.ImuRec_ImuPitch/180*pi;
raw_imu(:,10) = data.ImuRec_ImuYaw/180*pi;

% DVL
raw_dvl(:,2) = data.DvlRec_VelocityX;
raw_dvl(:,3) = data.DvlRec_VelocityY;
raw_dvl(:,4) = data.DvlRec_VelocityZ;
raw_dvl(:,5) = data.DvlCommunicationRec_BottomStatus;
raw_dvl(:,6) = data.DvlCommunicationRec_VelocityErrorBottom;

% BARO
raw_baro(:,2) = zeros(s_data,1);

S = size(raw_imu,1)-Nb_end;
for k = Nb_wrong+1:S
    DATA_IMU(:,k-Nb_wrong) = raw_imu(k,:);
    DATA_DVL(:,k-Nb_wrong) = raw_dvl(k,:);
    DATA_BARO(:,k-Nb_wrong) = raw_baro(k,:);
end

save('data_output/DATA_IMU.mat' , 'DATA_IMU');
save('data_output/DATA_DVL.mat' , 'DATA_DVL');
save('data_output/DATA_BARO.mat' , 'DATA_BARO');

disp('Data successfully extracted.');

clear;