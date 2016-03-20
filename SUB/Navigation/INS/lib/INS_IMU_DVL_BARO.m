function INS_IMU_DVL_BARO(block)
%   Level-2 MATLAB file S-Function for INS_EKF_IMU_DVL_BARO:
%
%   Accronymes :
%   INS : Inertial Navigation System
%   EKF : Extended Kalman Filter
%   IMU : Inertial Measurement Unit
%   ACC : Accelerometer
%   GYR : Gyroscope
%   MAG : Magnetometer
%   DVL : Doppler Velocity Log
%
%   Autor: Adrien Kerroux : adrienkerroux@gmail.com
%   SONIA 2015
%   30 Janvier 2016
%
%   References:
%   1- Aided Navigation - GPS with High Rate Sensors - Jay A. Farrell - 2008
%   2- Estimation Techniques for Low-Cost Inertial Navigation - Eun-Hwan Shin - 2005
%   3- Système de Navigation Hybride GPS/INS - Maîtrise Philippe Lavoie - 2012
%
    setup(block);
end

function setup(block)

    global dt dt_IMU dt_MAG dt_DVL dt_BARO;

    %% Register number of input port and output port
    block.NumInputPorts  = 5;
    block.NumOutputPorts = 4;

    %% Setup functional port properties
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    %% Inputs

    % Acc and Gyro
    block.InputPort(1).SampleTime  = [dt_IMU 0];
    block.InputPort(1).Dimensions    = 6;
    block.InputPort(1).SamplingMode = 'Sample';
    block.InputPort(1).DirectFeedthrough = false;

    
    % Mag
    block.InputPort(2).SampleTime  = [dt_MAG 0];
    block.InputPort(2).Dimensions    = 3;
    block.InputPort(2).SamplingMode = 'Sample';
    block.InputPort(2).DirectFeedthrough = false;
    
    % DVL
    block.InputPort(3).SampleTime  = [dt_DVL 0];
    block.InputPort(3).Dimensions    = 3;
    block.InputPort(3).SamplingMode = 'Sample';
    block.InputPort(3).DirectFeedthrough = false;
    
    % Baro
    block.InputPort(4).SampleTime  = [dt_BARO 0];
    block.InputPort(4).Dimensions    = 1;
    block.InputPort(4).SamplingMode = 'Sample';
    block.InputPort(4).DirectFeedthrough = false;
    
    % Time
    block.InputPort(5).SampleTime  = [dt 0];
    block.InputPort(5).Dimensions    = 1;
    block.InputPort(5).SamplingMode = 'Sample';
    block.InputPort(5).DirectFeedthrough = false;

    %% Outputs

    % State vector
    block.OutputPort(1).Dimensions   = 17;
    block.OutputPort(1).SamplingMode = 'Sample';  
    block.OutputPort(1).SampleTime = [dt 0];
    
    % Kalman State vector
    block.OutputPort(2).Dimensions   =16;
    block.OutputPort(2).SamplingMode = 'Sample';  
    block.OutputPort(2).SampleTime = [dt 0];
    
    % Residuals
    block.OutputPort(3).Dimensions   = 10;
    block.OutputPort(3).SamplingMode = 'Sample';  
    block.OutputPort(3).SampleTime = [dt 0];
    
    % Measurements variances
    block.OutputPort(4).Dimensions   = 10;
    block.OutputPort(4).SamplingMode = 'Sample';  
    block.OutputPort(4).SampleTime = [dt 0];

    %% Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';


    %% Reg methods
    block.RegBlockMethod('InitializeConditions',      @InitConditions);  
    block.RegBlockMethod('SetOutputPortSampleTime',	@SetOutPortST);
    block.RegBlockMethod('SetInputPortSampleTime',	@SetInpPortST);
    block.RegBlockMethod('SetInputPortSamplingMode',  @SetInpPortSM);
    block.RegBlockMethod('PostPropagationSetup',      @DoPostPropSetup);
    block.RegBlockMethod('Update',                    @Update);  
    block.RegBlockMethod('Outputs',                   @Output);  

end

function SetInpPortSM(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(idx).SamplingMode = fd;

    block.OutputPort(1).SamplingMode = fd;
    block.OutputPort(2).SamplingMode = fd;
    block.OutputPort(3).SamplingMode = fd;
    block.OutputPort(4).SamplingMode = fd;
end

% Set input port sample time
function SetInpPortST(block, idx, st)

    global dt dt_MAG dt_IMU dt_DVL dt_BARO;

    block.InputPort(1).SampleTime = [dt_IMU 0]; % acc gyro
    block.InputPort(2).SampleTime = [dt_MAG 0]; % mag
    block.InputPort(3).SampleTime = [dt_DVL 0]; % dvl
    block.InputPort(4).SampleTime = [dt_BARO 0]; % baro
    block.InputPort(5).SampleTime = [dt 0]; % time
    
end

% Set output port sample time
function SetOutPortST(block, idx, st)

    global dt;

    block.OutputPort(1).SampleTime = [dt 0]; % X
    block.OutputPort(2).SampleTime = [dt 0]; % dX
    block.OutputPort(3).SampleTime = [dt 0]; % dZ
    block.OutputPort(4).SampleTime = [dt 0]; % Zvar
    
end


% Do post-propagation process
function DoPostPropSetup(block)
    %% Setup DWork
    block.NumDworks = 4;

    % States
    block.Dwork(1).Name = 'X';
    block.Dwork(1).Dimensions      = 17;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 0;
    block.Dwork(1).UsedAsDiscState = true;

    % delta STATES
    block.Dwork(2).Name = 'dX';
    block.Dwork(2).Dimensions      = 16;
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 0;
    block.Dwork(2).UsedAsDiscState = true;
    
    % Residuals
    block.Dwork(3).Name = 'dZ';
    block.Dwork(3).Dimensions      = 10;
    block.Dwork(3).DatatypeID      = 0;
    block.Dwork(3).Complexity      = 0;
    block.Dwork(3).UsedAsDiscState = true;
    
    % Measurement Variances
    block.Dwork(4).Name = 'Zvar';
    block.Dwork(4).Dimensions      = 10;
    block.Dwork(4).DatatypeID      = 0;
    block.Dwork(4).Complexity      = 0;
    block.Dwork(4).UsedAsDiscState = true;

end

function InitConditions(block)
 
    global X0;

    block.Dwork(1).Data = X0;    
    block.Dwork(2).Data = zeros(16,1);
    block.Dwork(3).Data = zeros(10,1);
    block.Dwork(4).Data = zeros(10,1);
  
end
  
function Output(block)

    if block.OutputPort(1).IsSampleHit
        block.OutputPort(1).Data = block.Dwork(1).Data;
    end
    if block.OutputPort(2).IsSampleHit
        block.OutputPort(2).Data = block.Dwork(2).Data;
    end
    if block.OutputPort(3).IsSampleHit
        block.OutputPort(3).Data = block.Dwork(3).Data;
    end
    if block.OutputPort(4).IsSampleHit
        block.OutputPort(4).Data = block.Dwork(4).Data;
    end
    
end


function Update(block)
    
    global ge;
    global dt dt_MAG dt_DVL dt_BARO;
    global l_pD l_pp;
    global heading_shift_DVL;
    global CRIT_STATION_ACC CRIT_STATION_NORM;
    global SIGMA_MEAS_GRAVITY;
    global SIGMA_MEAS_MAG;
    global SIGMA_MEAS_DVL_X;
    global SIGMA_MEAS_DVL_Y;
    global SIGMA_MEAS_DVL_Z;
    global SIGMA_MEAS_BARO;
    global P0 Qc;
    global ACTIVE;
    
    persistent P;
    persistent magHasNewData dvlHasNewData baroHasNewData;
    persistent timeIMU timeMAG timeDVL timeBARO;
    persistent i;
    
    
    if( isempty(P) )
        i = 0;
        P = P0;
        timeIMU = 0;
        timeMAG = 0;
        timeDVL = 0;
        timeBARO = 0;
    end
    
    time = block.InputPort(5).Data;
    
    % Check measurements
    dvlHasNewData = 0;
    baroHasNewData = 0;
    
    if mod(time,dt_MAG) < dt
        magHasNewData = 1; % ATTENTION
    end
    
    if mod(time,dt_DVL) < dt
        dvlHasNewData = 1; % ATTENTION
    end
    
    if mod(time,dt_BARO) < dt
        baroHasNewData = 1; % ATTENTION
    end

    if(block.InputPort(1).IsSampleHit)
        i = i + 1;
        dtcalcIMU = time - timeIMU;
        timeIMU = time;

        % States extraction
        pos_n = block.Dwork(1).Data(1:3);
        vel_n = block.Dwork(1).Data(4:6);
        b_k_hat = block.Dwork(1).Data(7:10); % Quaternions
        bias_acc = block.Dwork(1).Data(11:13);
        bias_gyr = block.Dwork(1).Data(14:16);
        bias_baro = block.Dwork(1).Data(17);
        
        % Inertial data extraction And negative it
        ACC_GYRO = block.InputPort(1).Data;

        % IMU Processing
        f_b = ACC_GYRO(1:3) - bias_acc; % specific force
        w_ib_b = ACC_GYRO(4:6) - bias_gyr; % angular rates
        
        % Propagates exact quaternion
        b_k = exact_quat(w_ib_b,dt,b_k_hat);

        % Estimation of rotation matrix navigation frame to body frame
        R_n_b = quat2Rot(b_k); % Transform from quaternion to DCM
        R_b_n = R_n_b'; % Transposed

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% INS Mechanization Equations %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Position kinematic equation (12.20) - Farrell
        p_dot_n = vel_n;

        % Gravity in navigation frame
        g_n = [0; 0; ge];
        
        % Velocity kinematic equation (12.2) - Farrell
        v_dot_n = R_b_n*f_b + g_n; % We assume no corriolis effect ( w_in_n = 0 );
        % Integrate numericaly using Euler except for quaternion integrated
        % using an exact solution and bias which are not propagated
        pos_n = pos_n + p_dot_n*dt;
        vel_n = vel_n + v_dot_n*dt;
        
        % Reload state
        X_k = [pos_n;vel_n;b_k;bias_acc;bias_gyr;bias_baro];

        %% Matrix F Elements with dx_dot = F*dx + G*w
        % F_vp, F_rp and w_ie neglected
        
        F_pv = eye(3);
        
        F_vr = skew_matrix(g_n);
        
        F_vbf = -R_b_n;
        
        F_rbg = -R_b_n;
        
        F = [   zeros(3,3)	F_pv      	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(3,3)	zeros(3,3)	F_vr      	F_vbf       zeros(3,3) 	zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	F_rbg       zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(1,3) 	zeros(1,3)  zeros(1,3)  zeros(1,3)  zeros(1,3)  0];
        
        G = [
            zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,1);
            -R_b_n     zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,1);
            zeros(3,3) -R_b_n     zeros(3,3) zeros(3,3) zeros(3,1);
            zeros(3,3) zeros(3,3) eye(3,3)   zeros(3,3) zeros(3,1);
            zeros(3,3) zeros(3,3) zeros(3,3) eye(3,3)   zeros(3,1);
            zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,3) 1];

        %%%%%%%%%%%%%%%%%%%
        %% KALMAN FILTER %%
        %%%%%%%%%%%%%%%%%%%

        % Discretization
        G_k = G;
        Q = G_k*Qc*G_k'; % Equation (5.14) - Lavoie
        ufw = norm(w_ib_b)^2;
        Q_k = (Q+diag([ufw;ufw;ufw;0;0;0;0;0;0;0;0;0;0;0;0;0]))*dt;% 1st order Taylor series(Lavoie - Page 94)
        
        Phi_k = eye(length(F)) + (dt*F)^1/factorial(1); 

        P = Phi_k*P*Phi_k' + Q_k;
        
        %%%%%%%%%%%%
        %% UPDATE %%
        %%%%%%%%%%%%
        
        % test of stationarity of system
        ufab = norm(R_b_n*f_b + g_n,'fro');
        ufan = norm(f_b) - ge;
        if  ufab < CRIT_STATION_ACC && ufan < CRIT_STATION_NORM
            test_stationary = 1;
        else
            test_stationary = 0;
        end

        if ( (test_stationary == ACTIVE(1)) || (magHasNewData == ACTIVE(2)) || (dvlHasNewData == ACTIVE(3)) || (baroHasNewData == ACTIVE(4)) ) % Apply correction when new measurements are available from DVL, MAG, BARO
            
            if test_stationary == ACTIVE(1)
                %disp('gravity');
                
                skew_g_n = skew_matrix(g_n); % Equation 10.42 - Farrell
                H_GRAVITY = [zeros(3,3) zeros(3,3) -skew_g_n zeros(3,3) zeros(3,3) zeros(3,1)];
                R_GRAVITY = SIGMA_MEAS_GRAVITY^2*(ufab + ufan + ufw)*eye(3);
                
                K_GRAVITY = P*H_GRAVITY' / (H_GRAVITY*P*H_GRAVITY' + R_GRAVITY);
                P = (eye(16)-K_GRAVITY*H_GRAVITY)*P;
                
                z_gravity_hat = -R_b_n*f_b;
                z_gravity = g_n;
                d_z_gravity = z_gravity-z_gravity_hat;
                
                d_X = K_GRAVITY * d_z_gravity;
                [X_k, R_b_n] = updateStates(X_k, d_X);
                R_n_b = R_b_n';
            end
            
            if magHasNewData == ACTIVE(2)
                dtcalcMAG = time - timeMAG;
                timeMAG = time;
                
                [phi,theta,psi]=quat2euler(X_k(7:10));
                roll = phi;
                pitch = theta;
                yaw_hat = psi;
                
                R_b2w = [cos(pitch) , sin(pitch)*sin(roll) , sin(pitch)*cos(roll) ;
                0       ,      cos(roll)       ,     -sin(roll)       ;
                -sin(pitch) , cos(pitch)*sin(roll) , cos(pitch)*cos(roll)];

                % Code to calculate headingS
                m_b = block.InputPort(2).Data;
                m_w = R_b2w * m_b;
                yaw = atan2(-m_w(2) , m_w(1));
                
                Omega_T = [cos(psi)*cos(theta)  -sin(psi)   0;
                    sin(psi)*cos(theta)         cos(psi)    0;
                    -sin(theta)                 0           1];
                inv_Omega_T = inv(Omega_T);
                
                H_MAG = [zeros(1,3) zeros(1,3) inv_Omega_T(3,:) zeros(1,3) zeros(1,3) zeros(1,1)];
                R_MAG = SIGMA_MEAS_MAG^2;
                K_MAG = P*H_MAG' / (H_MAG*P*H_MAG' + R_MAG);
                
                d_z_mag = yaw - yaw_hat;
                
                if abs(d_z_mag) < pi
                    d_X = K_MAG * d_z_mag;
                    [X_k, R_b_n] = updateStates(X_k, d_X);
                    R_n_b = R_b_n';
                    P = (eye(16)-K_MAG*H_MAG)*P;
                end

                
                magHasNewData = 0;
            end
            
            if dvlHasNewData == ACTIVE(3)
                dtcalcDVL = time - timeDVL;
                timeDVL = time;
                
                DVL = block.InputPort(3).Data; % Extract measurements
                
                if norm(DVL) > 0 % Avoid corrections from NaN replaced by zeros
                    % Correct for heading shift
                    Rb2DVL = [cos(heading_shift_DVL) , - sin(heading_shift_DVL) , 0;
                    sin(heading_shift_DVL) , cos(heading_shift_DVL) , 0;
                    0         , 0                               , 1   ];

                    DVL = Rb2DVL'*DVL;

                    % Aiding Measurement Model
                    skew_l_pD = skew_matrix(l_pD);
                    H_DVL = [zeros(3,3) R_n_b zeros(3,3) zeros(3,3) skew_l_pD zeros(3,1)];
                    R_DVL = diag([SIGMA_MEAS_DVL_X SIGMA_MEAS_DVL_Y SIGMA_MEAS_DVL_Z]);

                    S_DVL = H_DVL*P*H_DVL' + R_DVL;
                    K_DVL = P*H_DVL' / S_DVL;
                    P = (eye(16)-K_DVL*H_DVL)*P;
                    DVL_hat = R_n_b*vel_n + cross(w_ib_b,l_pD); % Prediction

                    d_z_dvl = DVL - DVL_hat;
                    d_X = K_DVL * d_z_dvl;
                    [X_k, R_b_n] = updateStates(X_k, d_X);
                end

                dvlHasNewData = 0;
            end
            
            if baroHasNewData == ACTIVE(4)
                dtcalcBARO = time - timeBARO;
                timeBARO = time;
                
                baro_meas = block.InputPort(4).Data - bias_baro;
                
                if baro_meas > 100000 % is underwater ? [Pa]
                    
                    depth_meas = depth_calc(baro_meas); % Extract measurements

                    % Aiding Measurement Model
                    l_tp = R_b_n * l_pp;
                    skew_l_tp = skew_matrix(l_tp);
                    H_BARO = [0 0 1 zeros(1,3) -[0 0 1]*skew_l_tp zeros(1,3) zeros(1,3) 1/(1000*ge)];
                    R_BARO = SIGMA_MEAS_BARO;

                    depth_hat = pos_n(3); % Prediction
                    d_z_baro = depth_meas - depth_hat;
                    
                    K_BARO = P*H_BARO' / (H_BARO*P*H_BARO' + R_BARO);
                    P = (eye(16)-K_BARO*H_BARO)*P;
                    
                    d_X = K_BARO * d_z_baro;
                    [X_k, R_b_n] = updateStates(X_k, d_X);
                    
                end
                
                baroHasNewData = 0;
            end
                
            P =(P+P')/2; % Symmetrization
            
            block.Dwork(2).Data= zeros(16,1);
        else
            block.Dwork(2).Data= zeros(16,1);
        end
        block.Dwork(1).Data = X_k;

    end

end

%%%%%%%%%%%%%%%
%% Functions %%
%%%%%%%%%%%%%%%

% Function - Skew symmetric matrix
function Matrix = skew_matrix(vector)

    % Calculate the skew symmetric matrix of a vector
    Matrix = zeros(3,3);
    Matrix(1,2) = - vector(3);
    Matrix(2,1) = + vector(3);
    Matrix(1,3) = + vector(2);
    Matrix(3,1) = - vector(2);
    Matrix(2,3) = - vector(1);
    Matrix(3,2) = + vector(1);

end

% Convert quaternion to DCM
function [Rn2b]=quat2Rot(b)

    Bv = zeros(3,1);

    if norm(b)~= 0,
        b = b/norm(b); % Normalization of quaternion
        B = b(1);
        Bv(:,1)= b(2:4);
        Bc = [0    -Bv(3) Bv(2)
            Bv(3)  0    -Bv(1)
            -Bv(2)  Bv(1) 0];

        Rn2b = (B*B-Bv'*Bv)*eye(3,3)+2*(Bv*Bv')+2*B*Bc;
    else
        error('Norm quaternion = 0 in  quat2R function');
    end

end

% Convert quaternion to euler
function [phi,theta,psi]=quat2euler(b)

    phi = atan2(2*(b(3)*b(4) - b(1)*b(2)),1 - 2*(b(2)^2 + b(3)^2));
    theta = asin(-2*(b(2)*b(4) + b(1)*b(3)));
    psi = atan2(2*(b(2)*b(3) - b(1)*b(4)),1 - 2*(b(3)^2 + b(4)^2));

end

% Convert euler to DCM
function [Rn2b]=euler2Rot(roll,pitch,yaw)
    Rb2n = [cos(pitch)*cos(yaw) , sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) , cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
        cos(pitch)*sin(yaw) , sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) , cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
        -sin(pitch)         , sin(roll)*cos(pitch)                               , cos(roll)*cos(pitch)                             ];
    Rn2b = Rb2n';
end

% Convert DCM to quaternion
function [b]=Rot2Quat(R)

R = R +  (eye(3) - R*R') * 0.5 * R;

M1 = 1 + R(1,1) + R(2,2) + R(3,3);
M2 = 1 + R(1,1) - R(2,2) - R(3,3);
M3 = 1 - R(1,1) + R(2,2) - R(3,3);
M4 = 1 - R(1,1) - R(2,2) + R(3,3);
b = zeros(4,1);

if( (M1 > M2) && (M1 > M3)  &&  (M1 > M4))
    
    if(M1 > 0)
        b(1,1)    = (0.5*sqrt(M1));
        b(2,1)    = ( (R(3,2)-R(2,3))/(4*b(1)) );
        b(3,1)    = ( (R(1,3)-R(3,1))/(4*b(1)) );
        b(4,1)    = ( (R(2,1)-R(1,2))/(4*b(1)) );
        b         = (b/norm(b,'fro')); % renormalize
    else
        error('M1 won under 0');
    end
    
elseif( (M2 > M3) && (M2 > M4))
    
    if(M2 > 0)
        b(2,1) = 0.5*sqrt(M2);
        b(3,1) = (R(2,1) + R(1,2))/(4*b(2));
        b(4,1) = (R(3,1) + R(1,3))/(4*b(2));
        b(1,1) = (R(3,2) - R(2,3))/(4*b(2));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M2 won under 0');
    end
    
elseif( M3 > M4 )
    
    
    if(M3 > 0)
        b(3,1) = 0.5*sqrt(M3);
        
        b(2,1) = (R(1,2) + R(2,1))/(4*b(3));
        b(4,1) = (R(3,2) + R(2,3))/(4*b(3));
        b(1,1) = (R(1,3) - R(3,1))/(4*b(3));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M3 won under 0');
    end
    
else
    
    if(M4 > 0)
        b(4,1) = 0.5*sqrt(M4);               
        
        b(2,1) = (R(1,3) + R(3,1))/(4*b(4));
        b(3,1) = (R(2,3) + R(3,2))/(4*b(4));
        b(1,1) = (R(2,1) - R(1,2))/(4*b(4));
        b      = (b/norm(b,'fro')); % renormalize
    else
        error('M4 won under 0');
    end
    
end

end

% Function - Exact integration of quaternion

function b_exact = exact_quat(w_ib_b,dt,b_k)

    w_bn_b = - w_ib_b; % Equation (10.24) - Farrell (w_in_b assumed to be 0)

    % Integration of quaternion derivative (Equation (D.36) - Farrell)
    w = 0.5 * w_bn_b * dt;

    N = norm(w);

    if abs(N)>1
        error('Integrated angle too large');
    end

    skew_w = skew_matrix(w);

    W = [0   -w'
        w skew_w];

    % Handle singularity
    if N == 0
        sinw = 1;
    else
        sinw = sin(N)/N;
    end

    b_exact = (cos(N)*eye(4) + sinw*W)*b_k; % Equation (D.36) - Farrell

end



% Function - Calculate barometric depth

function depth = depth_calc(BARO)

    global ge;
    global SurfacePressure AirTemperature;
    
    if BARO > 101325 % positive depth (underwater)

        P0 = SurfacePressure; % Pressure init at surface [Pa]

        rho_water = 1000; % [kh/m3]

        depth = (BARO-P0) / ( rho_water * ge ); % Saunder-Fofonoff equation
        
    else % negative depth (outOfWater)
        
        h_s = 0;
        P_s = SurfacePressure/100; % Pressure in mbar
        T_s = AirTemperature; % Temperature in Kelvin
        P_B = BARO/100; % Pressure in mbar

        k_T = 6.5e-3; % K/m
        g0 = 9.8065; % m/s^2
        R = 287.1; % J/kg/K

        depth = -T_s/k_T*((P_B/P_s).^(-R*k_T/g0)-1) + h_s;
    end

end


function [X_k_update, R_b_n] = updateStates(X_k, dX)
    
    X_k_update = zeros(17,1);

    X_k_update(1:3) = X_k(1:3) + dX(1:3);
    X_k_update(4:6) = X_k(4:6) + dX(4:6);
    X_k_update(11:13) = X_k(11:13) + dX(10:12);
    X_k_update(14:16) = X_k(14:16) + dX(13:15);
    X_k_update(17) = X_k(17) + dX(16);
    
    b_k = X_k(7:10);
    R_n_b_hat = quat2Rot(b_k); % Transform from quaternion to DCM
    R_b_n_hat = R_n_b_hat'; % Transpose rotation matrix
    rho = dX(7:9); % Extract small-angle rotations vector
    Prho = skew_matrix(rho); % Equation (10.28) - Farrell
    R_b_n = (eye(3) + Prho)*R_b_n_hat; % Apply correction - Equation (10.32) - Farrell
    R_n_b = R_b_n'; % Transpose rotation matrix
    b_k_plus_one = Rot2Quat(R_n_b); % Transform from DCM to quaternion
    X_k_update(7:10) = b_k_plus_one;
    
end