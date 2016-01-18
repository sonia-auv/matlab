%%%%%%%%%%%%%%%%%%%
%% GLOBAL VALUES %%
%%%%%%%%%%%%%%%%%%%

% CONSTANT VALUES
global ge w_ie;
global dt dt_IMU dt_DVL dt_BARO;
global me_X me_Y me_Z meTOT me_dec me_inc;
% AUV PHYSICS
global l_pD l_pp;
% INITIALIZATION
global X0;
% QUASI-STATIC STATE DETECTION
global TIME_STATIC CRIT_STATIC_ACC CRIT_STATIC_GYRO;
% KALMAN MEASUREMENTS VARIANCES
global SIGMA_MEAS_IMU_ROLL;
global SIGMA_MEAS_IMU_PITCH;
global SIGMA_MEAS_IMU_YAW;
global SIGMA_MEAS_DVL_X;
global SIGMA_MEAS_DVL_Y;
global SIGMA_MEAS_DVL_Z;
global SIGMA_MEAS_BARO;
global SIGMA_MEAS_STATIC;
% KALMAN MATRIX CONSTANTS AND INITIALIZATION
global P0 Qc;
% CONTROLS
global ACTIVE;

%%%%%%%%%%%%%%%%%%%%%%%
%% CONTROLS & TUNING %%
%%%%%%%%%%%%%%%%%%%%%%%

% Controls
active_IMU = -1;
active_DVL = -1;
active_BARO = -1;
active_static = -1;
ACTIVE = [active_IMU active_DVL active_BARO active_static];

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurements variances %
%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMU Quaternions measurements variances
SIGMA_MEAS_IMU_ROLL = 1;
SIGMA_MEAS_IMU_PITCH = 1;
SIGMA_MEAS_IMU_YAW = 1;
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
SIGMA0_BIAS_BARO = 0.001;

% Propagation uncertainty
SIGMA_MEAS_ACC = 10;
SIGMA_MEAS_GYR = 0.01;
SIGMA_WALK_BIAS_ACC = 0.001;
SIGMA_WALK_BIAS_GYR = 0.001;
SIGMA_WALK_BIAS_BARO = 0.001;

%%%%%%%%%%%%%%%%%%%%%
%% CONSTANT VALUES %%
%%%%%%%%%%%%%%%%%%%%%
ge = 9.8065; % gravity [m/s/s]
me_X = 0.17749; % Magnetic tangent X [Gauss]
me_Y = -0.046285; % Magnetic tangent Y [Gauss]
me_Z = 0.51005; % Magnetic tangent Z [Gauss]
meTOT = 0.54203; % Magnetic total intensity [Gauss]
me_dec = -0.25437; % local magnetic field declination [rad] (positive eastwards)
me_inc = 1.226; % local magnetic field inclination [rad] (positive downwards)
w_ie = 7.2921e-5; % earth angular speed [rad/s]

%%%%%%%%%%%%%%%%%
%% AUV PHYSICS %%
%%%%%%%%%%%%%%%%%
% Distance from IMU to devices in body frame (NED)
l_pD = [0;0;0.15]; % IMU - DVL
l_pp = [0;-0.10;-0.5]; % IMU - BARO

%%%%%%%%%%%
%% RATES %%
%%%%%%%%%%%
dt = 1/50;
dt_IMU = 1/50;
dt_DVL = 1/3.42;
dt_BARO = 1/100;
t_simulation = size(DATA_IMU,2)*dt;

%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION %%
%%%%%%%%%%%%%%%%%%%%
pos0 = [0;0;0]; % depth at surface water
v0 = [0;0;0];

% roll = -DATA_IMU(8,1);
% pitch = -DATA_IMU(9,1);
% yaw = -DATA_IMU(10,1);
% R0_b_n = [cos(pitch)*cos(yaw) , sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) , cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
%     cos(pitch)*sin(yaw) , sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) , cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
%     -sin(pitch)         , sin(roll)*cos(pitch)                               , cos(roll)*cos(pitch)                             ];
% R0_n_b = R0_b_n';
% b0 = Rot2Quat(R0_n_b);
 b0 = DATA_IMU(8:11,1);

acc_bias0 = [0;0;0];
gyro_bias0 = [0;0;0];
baro_bias0 = 0;

X0 = [pos0; v0; b0; acc_bias0; gyro_bias0; baro_bias0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STATIC STATE DETECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TIME_STATIC = 0.2; % Time required to admit static state [s] during window variance calculation
CRIT_STATIC_ACC = 1; % Tolerance threshold normSq accel [m/s/s]
CRIT_STATIC_GYRO = 0.02; % Tolerance threshold normSq gyro [rad/s]


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