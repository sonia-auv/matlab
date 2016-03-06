%%%%%%%%%%%%%%%%%%%
%% GLOBAL VALUES %%
%%%%%%%%%%%%%%%%%%%

% CONSTANT VALUES
global ge;
global SurfacePressure AirTemperature;
global dt dt_IMU dt_MAG dt_DVL dt_BARO;
% AUV PHYSICS
global l_pD l_pp;
global heading_shift_DVL;
% INITIALIZATION
global X0;
% QUASI-STATIC & STATIONNARY STATE DETECTION
global CRIT_STATION_ACC CRIT_STATION_NORM;
% KALMAN MEASUREMENTS VARIANCES
global SIGMA_MEAS_GRAVITY;
global SIGMA_MEAS_MAG;
global SIGMA_MEAS_DVL_X;
global SIGMA_MEAS_DVL_Y;
global SIGMA_MEAS_DVL_Z;
global SIGMA_MEAS_BARO;
% KALMAN MATRIX CONSTANTS AND INITIALIZATION
global P0 Qc;
% CONTROLS
global ACTIVE;

%%%%%%%%%%%%%%%%%%%%
%% TIMING & RATES %%
%%%%%%%%%%%%%%%%%%%%
t_init = 0.5;
t_start = max([DATA_IMU(1,1);DATA_MAG(1,1);DATA_DVL(1,1);DATA_BARO(1,1)])+t_init;
t_end = min([DATA_IMU(1,end);DATA_MAG(1,end);DATA_DVL(1,end);DATA_BARO(1,end)]);
dt_IMU = 1/100;
dt = dt_IMU;
dt_MAG = 1/100;
dt_DVL = 1/3.5;
dt_BARO = 0.072;
t_simulation = t_end-t_start

%%%%%%%%%%%%%%%%%%%%%%%
%% CONTROLS & TUNING %%
%%%%%%%%%%%%%%%%%%%%%%%

% Controls
active_gravity = 1; % 1 active / -1 not active
active_mag = 1;
active_DVL = -1;
active_BARO = 1;
ACTIVE = [active_gravity active_mag active_DVL active_BARO];

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurements variances %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravity measurements variances
SIGMA_MEAS_GRAVITY = 10;
% Magneto measurements variances
SIGMA_MEAS_MAG = 5;
% DVL measurements variances
SIGMA_MEAS_DVL_X = 30;
SIGMA_MEAS_DVL_Y = 30;
SIGMA_MEAS_DVL_Z = 40;
% BARO measurements variances
SIGMA_MEAS_BARO = 40;

% Kalman initialization states uncertainty
SIGMA0_POS_X = 0.01;
SIGMA0_POS_Y = 0.01;
SIGMA0_POS_Z = 0.01;
SIGMA0_VEL_X = 0.01;
SIGMA0_VEL_Y = 0.01;
SIGMA0_VEL_Z = 0.01;
SIGMA0_RHO_X = 0.01;
SIGMA0_RHO_Y = 0.01;
SIGMA0_RHO_Z = 0.01;
SIGMA0_BIAS_ACC = 0.001;
SIGMA0_BIAS_GYR = 0.001;
SIGMA0_BIAS_BARO = 0.001;

% Propagation uncertainty
SIGMA_MEAS_ACC = 10;
SIGMA_MEAS_GYR = 0.01;
SIGMA_WALK_BIAS_ACC = 0.001;
SIGMA_WALK_BIAS_GYR = 0.001;
SIGMA_WALK_BIAS_BARO = 0.001;

%%%%%%%%%%%%%%%%%
%% AUV PHYSICS %%
%%%%%%%%%%%%%%%%%
% Distance from IMU to devices in body frame (NED)
l_pD = [0;0;0.15]; % IMU - DVL
l_pp = [0;-0.10;-0.5]; % IMU - BARO
heading_shift_DVL = -45*pi/180; % Heading shift DVL [rad]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STATIONNARY STATE DETECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CRIT_STATION_ACC = 1;
CRIT_STATION_NORM = 1;

%%%%%%%%%%%%%%%%%%%%%
%% DATA EXTRACTION %%
%%%%%%%%%%%%%%%%%%%%%
start_record_IMU = find(DATA_IMU(1,:) >= t_start,1,'first');
final_record_IMU = find(DATA_IMU(1,:) >= t_end,1,'first');
Sensors_IMU = [
    DATA_IMU(1,start_record_IMU:final_record_IMU)- t_start;
    DATA_IMU(2,start_record_IMU:final_record_IMU); % AccX
    -DATA_IMU(3,start_record_IMU:final_record_IMU); % AccY - IMU reversed on sub
    -DATA_IMU(4,start_record_IMU:final_record_IMU); % AccZ - IMU reversed on sub
    DATA_IMU(5,start_record_IMU:final_record_IMU); % GyrX
    -DATA_IMU(6,start_record_IMU:final_record_IMU); % GyrY - IMU reversed on sub
    -DATA_IMU(7,start_record_IMU:final_record_IMU); % GyrZ - IMU reversed on sub
    ];
for i = 1:size(Sensors_IMU,1)
    for j = 1:size(Sensors_IMU,2)
        if isnan(Sensors_IMU(i,j))
            strI = strcat(' i=',num2str(i));
            strJ = strcat(' j=',num2str(j));
            disp(strcat(strcat('ERROR NAN IN SENSORS IMU FOR ',strcat(strI,strJ)),' replace by 0'));
            Sensors_IMU(i,j) = 0;
        end
    end
end
save Sensors_IMU Sensors_IMU

start_record_MAG = find(DATA_MAG(1,:) >= t_start,1,'first');
final_record_MAG = find(DATA_MAG(1,:) >= t_end,1,'first');
Sensors_MAG = [
    DATA_MAG(1,start_record_MAG:final_record_MAG)- t_start;
    DATA_MAG(2,start_record_MAG:final_record_MAG); % MagX
    -DATA_MAG(3,start_record_MAG:final_record_MAG); % MagY - IMU reversed on sub
    -DATA_MAG(4,start_record_MAG:final_record_MAG); % MagZ - IMU reversed on sub
    ];
for i = 1:size(Sensors_MAG,1)
    for j = 1:size(Sensors_MAG,2)
        if isnan(Sensors_MAG(i,j))
            strI = strcat(' i=',num2str(i));
            strJ = strcat(' j=',num2str(j));
            disp(strcat(strcat('ERROR NAN IN SENSORS MAG FOR ',strcat(strI,strJ)),' replace by 0'));
            Sensors_MAG(i,j) = 0;
        end
    end
end
save Sensors_MAG Sensors_MAG

start_record_DVL = find(DATA_DVL(1,:) >= t_start,1,'first');
final_record_DVL = find(DATA_DVL(1,:) >= t_end,1,'first');
Sensors_DVL = [
    DATA_DVL(1,start_record_DVL:final_record_DVL)- t_start;
    DATA_DVL(2,start_record_DVL:final_record_DVL); % DvlX
    DATA_DVL(3,start_record_DVL:final_record_DVL); % DvlY
    DATA_DVL(4,start_record_DVL:final_record_DVL); % DvlZ
    ];
for i = 1:size(Sensors_DVL,1)
    for j = 1:size(Sensors_DVL,2)
        if isnan(Sensors_DVL(i,j))
            strI = strcat(' i=',num2str(i));
            strJ = strcat(' j=',num2str(j));
            disp(strcat(strcat('ERROR NAN IN SENSORS DVL FOR ',strcat(strI,strJ)),' replace by 0'));
            Sensors_DVL(i,j) = 0;
        end
    end
end
save Sensors_DVL Sensors_DVL

start_record_BARO = find(DATA_BARO(1,:) >= t_start,1,'first');
final_record_BARO = find(DATA_BARO(1,:) >= t_end,1,'first');
Sensors_BARO = [
    DATA_BARO(1,start_record_DVL:final_record_DVL)- t_start;
    DATA_BARO(2,start_record_DVL:final_record_DVL); % Baro [Pa]
    ];
for i = 1:size(Sensors_BARO,1)
    for j = 1:size(Sensors_BARO,2)
        if isnan(Sensors_BARO(i,j))
            strI = strcat(' i=',num2str(i));
            strJ = strcat(' j=',num2str(j));
            disp(strcat(strcat('ERROR NAN IN SENSORS BARO FOR ',strcat(strI,strJ)),' replace by 0'));
            Sensors_BARO(i,j) = 0;
        end
    end
end
save Sensors_BARO Sensors_BARO


%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION %%
%%%%%%%%%%%%%%%%%%%%

start_init_IMU = find(DATA_IMU(1,:) >= (t_start-t_init),1,'first');
gx_mean = mean(-DATA_IMU(2,start_init_IMU:start_record_IMU-1));
gy_mean = mean(DATA_IMU(3,start_init_IMU:start_record_IMU-1)); % ATTENTION IMU inverted
gz_mean = mean(DATA_IMU(4,start_init_IMU:start_record_IMU-1)); % ATTENTION IMU inverted
%ge = norm([gx_mean;gy_mean;gz_mean])
ge = 9.8;

roll  = atan2(gy_mean,gz_mean) % Calculate initial roll angle - Equation 10.14 - Farrell
pitch = atan2(-gx_mean , sqrt(gy_mean^2 + gz_mean^2)) % Calculate initial pitch angle - Equation 10.15 - Farrell

% Calculate rotation matrix - Equation 10.16 - Farrell
R_b2w = [cos(pitch) , sin(pitch)*sin(roll) , sin(pitch)*cos(roll) ;
    0       ,      cos(roll)       ,     -sin(roll)       ;
    -sin(pitch) , cos(pitch)*sin(roll) , cos(pitch)*cos(roll)];

% Calculate the mean of Magnetometer measurements - Equation 10.17 - Farrell
start_init_MAG = find(DATA_MAG(1,:) >= (t_start-t_init),1,'first');
mx_mean = mean(DATA_MAG(2,start_init_MAG:start_record_MAG-1)); % X axis in Gauss
my_mean = mean(-DATA_MAG(3,start_init_MAG:start_record_MAG-1)); % Y axis in Gauss % ATTENTION IMU inverted
mz_mean = mean(-DATA_MAG(4,start_init_MAG:start_record_MAG-1)); % Z axis in Gauss % ATTENTION IMU inverted

% Code to calculate headingS
m_b = [mx_mean;my_mean;mz_mean];

m_w = R_b2w * m_b;

yaw = atan2(-m_w(2) , m_w(1));

R0_b_n = [cos(pitch)*cos(yaw) , sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) , cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw) , sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) , cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    -sin(pitch)         , sin(roll)*cos(pitch)                               , cos(roll)*cos(pitch)                             ];
R0_n_b = R0_b_n';
b0 = Rot2Quat(R0_n_b);

% Estimation init depth
SurfacePressure = 101325; % [Pa]
AirTemperature = 298.15; % [K] (25 degrees Celcius)

start_init_BARO = find(DATA_BARO(1,:) >= (t_start-t_init),1,'first');
baro_mean = mean(DATA_BARO(2,start_init_BARO:start_record_BARO-1));

if baro_mean > 101325 % positive depth (underwater)

    P0 = SurfacePressure; % Pressure init at surface [Pa]

    rho_water = 1000; % [kh/m3]

    depth0 = (baro_mean-P0) / ( rho_water * ge ); % Saunder-Fofonoff equation

else % negative depth (outOfWater)

    h_s = 0;
    P_s = SurfacePressure/100; % Pressure in mbar
    T_s = AirTemperature; % Temperature in Kelvin
    P_B = baro_mean/100; % Pressure in mbar

    k_T = 6.5e-3; % K/m
    g0 = 9.8065; % m/s^2
    R = 287.1; % J/kg/K

    depth0 = -T_s/k_T*((P_B/P_s).^(-R*k_T/g0)-1) + h_s;
end

pos0 = [0;0;depth0]; % depth at surface water

% Estimate init velocity

if depth0 > 0
    
    yawDVL = yaw - heading_shift_DVL;

    RbDVL2n = [cos(pitch)*cos(yawDVL)   , sin(roll)*sin(pitch)*cos(yawDVL) - cos(roll)*sin(yawDVL)  , cos(roll)*sin(pitch)*cos(yawDVL) + sin(roll)*sin(yawDVL);
            cos(pitch)*sin(yawDVL)      , sin(roll)*sin(pitch)*sin(yawDVL) + cos(roll)*cos(yawDVL)  , cos(roll)*sin(pitch)*sin(yawDVL) - sin(roll)*cos(yawDVL);
            -sin(pitch)                 , sin(roll)*cos(pitch)                                      , cos(roll)*cos(pitch)                                   ];

    xDVL = DATA_DVL(2,start_record_BARO-1);
    yDVL = DATA_DVL(4,start_record_BARO-1);
    zDVL = DATA_DVL(4,start_record_BARO-1);
    v0 = RbDVL2n*[xDVL;yDVL;zDVL]
else
    v0 = [0;0;0];
end

acc_bias0 = [0;0;0];
gyro_bias0 = [0;0;0];
baro_bias0 = 0;

X0 = [pos0; v0; b0; acc_bias0; gyro_bias0; baro_bias0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EXTENDED KALMAN FILTER %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Qc = diag([SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;...
    SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;...
    SIGMA_WALK_BIAS_ACC;SIGMA_WALK_BIAS_ACC;SIGMA_WALK_BIAS_ACC;...
    SIGMA_WALK_BIAS_GYR;SIGMA_WALK_BIAS_GYR;SIGMA_WALK_BIAS_GYR;...
    SIGMA_WALK_BIAS_BARO]);

P0 = diag([SIGMA0_POS_X;SIGMA0_POS_Y;SIGMA0_POS_Z;...
    SIGMA0_VEL_X;SIGMA0_VEL_Y;SIGMA0_VEL_Z;...
    SIGMA0_RHO_X;SIGMA0_RHO_Y;SIGMA0_RHO_Z;...
    SIGMA0_BIAS_ACC;SIGMA0_BIAS_ACC;SIGMA0_BIAS_ACC;...
    SIGMA0_BIAS_GYR;SIGMA0_BIAS_GYR;SIGMA0_BIAS_GYR;...
    SIGMA0_BIAS_BARO]);
