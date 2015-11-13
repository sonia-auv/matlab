
% CONSTANT VALUES
global ge w_ie;
global dt dt_IMU dt_DVL dt_BARO;
global me_X me_Y me_Z meTOT me_dec me_inc;
% AUV PHYSICS
global l_pD l_pp;
% INITIALIZATION
global TIME_INIT X0;
% QUASI-STATIC STATE DETECTION
global TIME_STATIC TOLERANCE_ACC TOLERANCE_GYRO;
% KALMAN MEASUREMENTS CRITERION
global CRIT_EULER_PHI_THETA CRIT_EULER_PSI;
global CRIT_DVL_X_Y CRIT_DVL_Z CRIT_DVL_ERR;
global CRIT_BARO;
global CRIT_ZERO_UPDATE;
% KALMAN MATRIX CONSTANTS AND INITIALIZATION
global Qc P0;

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
dt = 1/200;
dt_IMU = 1/200;
dt_DVL = 1/7;
dt_BARO = 1/100;
t_simulation = size(DATA_IMU,2)*dt;

%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION %%
%%%%%%%%%%%%%%%%%%%%
TIME_INIT = 0; % Time required for initialization

pos0 = [0;0;0.1]; % depth at surface water
v0 = [0;0;0];

roll = DATA_IMU(8,1);
pitch = DATA_IMU(9,1);
yaw = DATA_IMU(10,1);
R0_b_n = [cos(pitch)*cos(yaw) , sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) , cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw) , sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) , cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
    -sin(pitch)         , sin(roll)*cos(pitch)                               , cos(roll)*cos(pitch)                             ];
R0_n_b = R0_b_n';
b0 = Rot2Quat(R0_n_b);

acc_bias0 = [0;0;0];
gyro_bias0 = [0;0;0];
baro_bias0 = 0;

X0 = [pos0; v0; b0; acc_bias0; gyro_bias0; baro_bias0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STATIC STATE DETECTION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TIME_STATIC = 0.2; % Time required to admit static state [s] during window variance calculation
TOLERANCE_ACC = 1; % Tolerance threshold normSq accel [m/s/s]
TOLERANCE_GYRO = 0.02; % Tolerance threshold normSq gyro [rad/s]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KALMAN MEASUREMENTS CRITERION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CRIT_EULER_PHI_THETA = 10;
CRIT_EULER_PSI = 100;
CRIT_DVL_X_Y = 5;
CRIT_DVL_Z = 10;
CRIT_DVL_ERR = 1;
CRIT_BARO = 10;
CRIT_ZERO_UPDATE = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% KALMAN MATRIX CONSTANTS AND INITIALIZATION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SIGMA_MEAS_ACC = 1;
SIGMA_MEAS_GYR = 5;
SIGMA_WALK_ACC = 0.05;
SIGMA_WALK_GYR = 0.01;
SIGMA_WALK_BARO = 0.1;

SIGMA_POS_XY = 100;
SIGMA_POS_Z = 50;
SIGMA_VEL_XY = 100;
SIGMA_VEL_Z = 50;
SIGMA_RHO_X = 10;
SIGMA_RHO_Y = 10;
SIGMA_RHO_Z = 10;

Qc = diag([SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;SIGMA_MEAS_ACC;...
    SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;SIGMA_MEAS_GYR;...
    SIGMA_WALK_ACC;SIGMA_WALK_ACC;SIGMA_WALK_ACC;...
    SIGMA_WALK_GYR;SIGMA_WALK_GYR;SIGMA_WALK_GYR;...
    SIGMA_WALK_BARO]);

P0 = diag([SIGMA_POS_XY;SIGMA_POS_XY;SIGMA_POS_Z;...
    SIGMA_VEL_XY;SIGMA_VEL_XY;SIGMA_VEL_Z;...
    SIGMA_RHO_X;SIGMA_RHO_Y;SIGMA_RHO_Z;...
    SIGMA_WALK_ACC;SIGMA_WALK_ACC;SIGMA_WALK_ACC;...
    SIGMA_WALK_GYR;SIGMA_WALK_GYR;SIGMA_WALK_GYR;...
    SIGMA_WALK_BARO]);
