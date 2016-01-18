function [] = process_imu( file_name )

Nb_wrong = 0; % Wrong data
Nb_end = 0;

ds = datastore(strcat(file_name,'.csv'),'Delimiter',',');
Variables = ds.VariableNames
ds.SelectedVariableNames = {'Timestamp',...
                            'q0', 'q1', 'q2', 'q3',...
                            'gyroX', 'gyroY', 'gyroZ',...
                            'accX', 'accY', 'accZ'};

data = readall(ds);
format long
start_time = data.Timestamp(1);

raw_time = (data.Timestamp-start_time)/1000000000;

%s_data = size(data.Timestamp, 1);

raw_imu(:,1) = raw_time;

raw_imu(:,2) = data.accX;
raw_imu(:,3) = data.accY;
raw_imu(:,4) = data.accZ;
raw_imu(:,5) = data.gyroX;
raw_imu(:,6) = data.gyroY;
raw_imu(:,7) = data.gyroZ;
raw_imu(:,8) = data.q0;
raw_imu(:,9) = data.q1;
raw_imu(:,10) = data.q2;
raw_imu(:,11) = data.q3;

S = size(raw_imu,1)-Nb_end;
for k = Nb_wrong+1:S
    DATA_IMU(:,k-Nb_wrong) = raw_imu(k,:);
end

save('data_output/DATA_IMU.mat' , 'DATA_IMU');





end

