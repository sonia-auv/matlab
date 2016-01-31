%%%%%%%%%%%%%%%%%%%
%% GLOBAL VALUES %%
%%%%%%%%%%%%%%%%%%%

% CONSTANT VALUES
global ge;
global dt dt_IMU dt_MAG dt_DVL dt_BARO;
global me me_dec me_inc;
% AUV PHYSICS
global l_pD l_pp;
% INITIALIZATION
global X0;
% QUASI-STATIC & STATIONNARY STATE DETECTION
global TIME_STATIC CRIT_STATIC_ACC CRIT_STATIC_GYRO;
global CRIT_STATION_ACC CRIT_STATION_NORM;
% KALMAN MEASUREMENTS VARIANCES
global SIGMA_MEAS_GRAVITY;
global SIGMA_MEAS_MAG;
global SIGMA_MEAS_DVL_X;
global SIGMA_MEAS_DVL_Y;
global SIGMA_MEAS_DVL_Z;
global SIGMA_MEAS_BARO;
global SIGMA_MEAS_STATIC;
% KALMAN MATRIX CONSTANTS AND INITIALIZATION
global P0 Qc;
% CONTROLS
global ACTIVE;

%%%%%%%%%%%%%%%%%%%%
%% TIMING & RATES %%
%%%%%%%%%%%%%%%%%%%%
t_init = 0.5;
t_start = max([DATA_IMU(1,1);DATA_MAG(1,1);DATA_DVL(1,1)])+t_init;
t_end = min([DATA_IMU(1,end);DATA_MAG(1,end);DATA_DVL(1,end)]);
dt = 1/100;
dt_IMU = 1/100;
dt_MAG = 1/100;
dt_DVL = 1/3.5;
dt_BARO = 1/100;
t_simulation = t_end-t_start

%%%%%%%%%%%%%%%%%%%%%%%
%% CONTROLS & TUNING %%
%%%%%%%%%%%%%%%%%%%%%%%

% Controls
active_gravity = -1;
active_DVL = -1;
active_BARO = -1;
active_static = -1;
ACTIVE = [active_gravity active_DVL active_BARO active_static];

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurements variances %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravity measurements variances
SIGMA_MEAS_GRAVITY = 1;
% Magneto measurements variances
SIGMA_MEAS_MAG = 1;
% DVL measurements variances
SIGMA_MEAS_DVL_X = 3;
SIGMA_MEAS_DVL_Y = 3;
SIGMA_MEAS_DVL_Z = 5;
% BARO measurements variances
SIGMA_MEAS_BARO = 10;
% Static State measurements variances
SIGMA_MEAS_STATIC = 1;

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
SIGMA0_BIAS_MAG = 0.001;
SIGMA0_BIAS_BARO = 0.001;

% Propagation uncertainty
SIGMA_MEAS_ACC = 10;
SIGMA_MEAS_GYR = 0.01;
SIGMA_WALK_BIAS_ACC = 0.001;
SIGMA_WALK_BIAS_GYR = 0.001;
SIGMA_WALK_BIAS_MAG = 0.001;
SIGMA_WALK_BIAS_BARO = 0.001;

%%%%%%%%%%%%%%%%%%%%%
%% CONSTANT VALUES %%
%%%%%%%%%%%%%%%%%%%%%
ge = 9.795; % gravity [m/s/s]
me_dec = 0.2034; % local magnetic field declination [rad] (positive eastwards)
me_inc = 1.009; % local magnetic field inclination [rad] (positive downwards)

%%%%%%%%%%%%%%%%%
%% AUV PHYSICS %%
%%%%%%%%%%%%%%%%%
% Distance from IMU to devices in body frame (NED)
l_pD = [0;0;0.15]; % IMU - DVL
l_pp = [0;-0.10;-0.5]; % IMU - BARO

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STATIC % STATIONNARY STATE DETECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TIME_STATIC = 0.2; % Time required to admit static state [s] during window variance calculation
CRIT_STATIC_ACC = 1; % Tolerance threshold normSq accel [m/s/s]
CRIT_STATIC_GYRO = 0.02; % Tolerance threshold normSq gyro [rad/s]
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
save Sensors_IMU Sensors_IMU

start_record_MAG = find(DATA_MAG(1,:) >= t_start,1,'first');
final_record_MAG = find(DATA_MAG(1,:) >= t_end,1,'first');
Sensors_MAG = [
    DATA_MAG(1,start_record_MAG:final_record_MAG)- t_start;
    DATA_MAG(2,start_record_MAG:final_record_MAG); % MagX
    -DATA_MAG(3,start_record_MAG:final_record_MAG); % MagY - IMU reversed on sub
    -DATA_MAG(4,start_record_MAG:final_record_MAG); % MagZ - IMU reversed on sub
    ];
save Sensors_MAG Sensors_MAG

start_record_DVL = find(DATA_DVL(1,:) >= t_start,1,'first');
final_record_DVL = find(DATA_DVL(1,:) >= t_end,1,'first');
Sensors_DVL = [
    DATA_DVL(1,start_record_DVL:final_record_DVL)- t_start;
    DATA_DVL(2,start_record_DVL:final_record_DVL); % DvlX
    -DATA_DVL(3,start_record_DVL:final_record_DVL); % DvlY
    -DATA_DVL(4,start_record_DVL:final_record_DVL); % DvlZ
    ];
save Sensors_DVL Sensors_DVL


%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION %%
%%%%%%%%%%%%%%%%%%%%
pos0 = [0;0;0]; % depth at surface water
v0 = [0;0;0];

start_init_IMU = find(DATA_IMU(1,:) >= (t_start-t_init),1,'first');
gx_mean = mean(-DATA_IMU(2,start_init_IMU:start_record_IMU-1));
gy_mean = mean(-DATA_IMU(3,start_init_IMU:start_record_IMU-1));
gz_mean = mean(-DATA_IMU(4,start_init_IMU:start_record_IMU-1));

roll  = atan2(gy_mean,gz_mean); % Calculate initial roll angle - Equation 10.14 - Farrell
pitch = atan2(-gx_mean , sqrt(gy_mean^2 + gz_mean^2)); % Calculate initial pitch angle - Equation 10.15 - Farrell

% Calculate rotation matrix - Equation 10.16 - Farrell
R_b2w = [cos(pitch) , sin(pitch)*sin(roll) , sin(pitch)*cos(roll) ;
    0       ,      cos(roll)       ,     -sin(roll)       ;
    -sin(pitch) , cos(pitch)*sin(roll) , cos(pitch)*cos(roll)];

% Calculate the mean of Magnetometer measurements - Equation 10.17 - Farrell
start_init_MAG = find(DATA_MAG(1,:) >= (t_start-t_init),1,'first');
mx_mean = mean(DATA_MAG(2,start_init_MAG:start_record_MAG-1)); % X axis in Gauss
my_mean = mean(DATA_MAG(3,start_init_MAG:start_record_MAG-1)); % Y axis in Gauss
mz_mean = mean(DATA_MAG(4,start_init_MAG:start_record_MAG-1)); % Z axis in Gauss

% Code to calculate headingS
m_b = [mx_mean;my_mean;mz_mean];
m_b_normal = m_b / norm(m_b,'fro');
me = norm(m_b); % Magnetic total intensity [Gauss]

m_w = R_b2w * m_b;

yaw = atan2(-m_w(2) , m_w(1));

R0_b_n = [cos(pitch)*cos(yaw) , sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) , cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw) , sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) , cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    -sin(pitch)         , sin(roll)*cos(pitch)                               , cos(roll)*cos(pitch)                             ];
R0_n_b = R0_b_n';
b0 = Rot2Quat(R0_n_b);

acc_bias0 = [0;0;0];
gyro_bias0 = [0;0;0];
mag_bias0 = [0;0;0];
baro_bias0 = 0;

X0 = [pos0; v0; b0; acc_bias0; gyro_bias0; mag_bias0; baro_bias0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EXTENDED KALMAN FILTER %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Qc = diag([SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;...
    SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;...
    SIGMA_WALK_BIAS_ACC;SIGMA_WALK_BIAS_ACC;SIGMA_WALK_BIAS_ACC;...
    SIGMA_WALK_BIAS_GYR;SIGMA_WALK_BIAS_GYR;SIGMA_WALK_BIAS_GYR;...
    SIGMA_WALK_BIAS_MAG;SIGMA_WALK_BIAS_MAG;SIGMA_WALK_BIAS_MAG;...
    SIGMA_WALK_BIAS_BARO]);

P0 = diag([SIGMA0_POS_X;SIGMA0_POS_Y;SIGMA0_POS_Z;...
    SIGMA0_VEL_X;SIGMA0_VEL_Y;SIGMA0_VEL_Z;...
    SIGMA0_RHO_X;SIGMA0_RHO_Y;SIGMA0_RHO_Z;...
    SIGMA0_BIAS_ACC;SIGMA0_BIAS_ACC;SIGMA0_BIAS_ACC;...
    SIGMA0_BIAS_GYR;SIGMA0_BIAS_GYR;SIGMA0_BIAS_GYR;...
    SIGMA0_BIAS_MAG;SIGMA0_BIAS_MAG;SIGMA0_BIAS_MAG;...
    SIGMA0_BIAS_BARO]);
