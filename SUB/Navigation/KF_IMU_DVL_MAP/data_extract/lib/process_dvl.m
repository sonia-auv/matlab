function [] = process_dvl( file_name )

Nb_wrong = 0; % Wrong data
Nb_end = 0;

ds = datastore(strcat(file_name,'.csv'));
Variables = ds.VariableNames
ds.SelectedVariableNames = {'Timestamp',...
                            'ImuRec_ImuRollAngularRate', 'ImuRec_ImuPitchAngularRate', 'ImuRec_ImuYawAngularRate',...
                            'ImuRec_ImuRoll', 'ImuRec_ImuPitch', 'ImuRec_ImuYaw',...
                            'DvlRec_VelocityX', 'DvlRec_VelocityY', 'DvlRec_VelocityZ',...
                            'DvlCommunicationRec_BottomStatus', 'DvlCommunicationRec_VelocityErrorBottom'};

data = readall(ds);
format long
start_time = data.Timestamp(1);

raw_time = (data.Timestamp-start_time)/1000;

%s_data = size(data.Timestamp, 1);

raw_dvl(:,1) = raw_time;

raw_dvl(:,2) = data.DvlRec_VelocityX;
raw_dvl(:,3) = data.DvlRec_VelocityY;
raw_dvl(:,4) = data.DvlRec_VelocityZ;
raw_dvl(:,5) = data.DvlCommunicationRec_BottomStatus;
raw_dvl(:,6) = data.DvlCommunicationRec_VelocityErrorBottom;

S = size(raw_imu,1)-Nb_end;
for k = Nb_wrong+1:S
    DATA_DVL(:,k-Nb_wrong) = raw_dvl(k,:);
end

save('data_output/DATA_DVL.mat' , 'DATA_DVL');



end

