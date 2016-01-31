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
    block.OutputPort(2).Dimensions   = 16;
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

    global dt dt_IMU dt_DVL dt_BARO;

    block.InputPort(1).SampleTime = [dt_IMU 0]; % acc gyro
    block.InputPort(2).SampleTime = [dt_IMU 0]; % quaternions
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
    
    global ge w_ie;
    global dt dt_IMU dt_DVL dt_BARO;
    global me_X me_Y me_Z meTOT me_dec me_inc;
    global l_pD l_pp;
    global X0;
    global TIME_STATIC CRIT_STATIC_ACC CRIT_STATIC_GYRO;
    global SIGMA_MEAS_IMU_ROLL;
    global SIGMA_MEAS_IMU_PITCH;
    global SIGMA_MEAS_IMU_YAW;
    global SIGMA_MEAS_DVL_X;
    global SIGMA_MEAS_DVL_Y;
    global SIGMA_MEAS_DVL_Z;
    global SIGMA_MEAS_BARO;
    global SIGMA_MEAS_STATIC;
    global P0 Qc;
    global ACTIVE;
    
    persistent P;
    persistent counter_static;
    persistent imuHasNewData dvlHasNewData baroHasNewData;
    
    if( isempty(P) )
        P = P0;
        counter_static = 0;
        
    end
    
    imuHasNewData = 0;
    dvlHasNewData = 0;
    baroHasNewData = 0;
    
    time = block.InputPort(5).Data;
    
    % Check measurements
    if mod(time,dt_IMU) == 0
        imuHasNewData = 1; % ATTENTION
    end
    
    if mod(time,dt_DVL) == 0
        dvlHasNewData = 1; % ATTENTION
    end
    
    if mod(time,dt_BARO) == 0
        baroHasNewData = 1; % ATTENTION
    end

    if(mod(time,dt) == 0)
        
        % States extraction
        pos_n = block.Dwork(1).Data(1:3);
        vel_b = block.Dwork(1).Data(4:6);
        b = block.Dwork(1).Data(7:10); % Quaternions
        bias_acc = block.Dwork(1).Data(11:13);
        bias_gyr = block.Dwork(1).Data(14:16);
        bias_baro = block.Dwork(1).Data(17);

        % Kalman states extraction
        d_pos = block.Dwork(2).Data(1:3);
        d_vel = block.Dwork(2).Data(4:6);
        rho = block.Dwork(2).Data(7:9); % small angles vector
        d_bias_acc = block.Dwork(2).Data(10:12);
        d_bias_gyr = block.Dwork(2).Data(13:15);
        d_bias_baro = block.Dwork(1).Data(16);
        
        % Inertial data extraction And negative it
        ACC_GYRO(1,1) = block.InputPort(1).Data(1);
        ACC_GYRO(2,1) = -block.InputPort(1).Data(2);
        ACC_GYRO(3,1) = -block.InputPort(1).Data(3);
        ACC_GYRO(4,1) = block.InputPort(1).Data(4);
        ACC_GYRO(5,1) = -block.InputPort(1).Data(5);
        ACC_GYRO(6,1) = -block.InputPort(1).Data(6);
        ACC_GYRO
        
        % Corrections with Kalman states
        pos_n = pos_n + d_pos;
        vel_b = vel_b + d_vel;
        bias_acc = bias_acc + d_bias_acc;
        bias_gyr = bias_gyr + d_bias_gyr;
        bias_baro = bias_baro + d_bias_baro;

        % IMU Processing
        f_b = ACC_GYRO(1:3) - bias_acc; % specific force
        w_ib_b = ACC_GYRO(4:6) - bias_gyr; % angular rates

        % Estimation of rotation matrix navigation frame to body frame
        R_n_b_hat = quat2Rot(b); % Transform from quaternion to DCM
        R_b_n_hat = R_n_b_hat'; % Transposed
        
        % Small angles matrix
        Prho = skew_matrix(rho); % Equation (10.28) - Farrell

        R_b_n = (eye(3) + Prho)*R_b_n_hat; % Equation (10.32) - Farrell
        R_n_b = R_b_n';
        b = Rot2Quat(R_n_b); % Transform from DCM to quaternion
        b = exact_quat(w_ib_b,dt,b); % Exact solution - Farrell - Page 508

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% INS Mechanization Equations %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Position kinematic equation (12.20) - Farrell
        p_dot_n = R_b_n*vel_b;

        % Gravity in navigation frame
        g_n = [0; 0; ge];
        
        % Velocity kinematic equation (12.2) - Farrell
        v_dot_b = f_b + R_n_b*g_n; % We assume no corriolis effect ( w_in_n = 0 );
        % Integrate numericaly using Euler except for quaternion integrated
        % using an exact solution and bias which are not propagated
        pos_n = pos_n + p_dot_n*dt;
        vel_b = vel_b + v_dot_b*dt;
        
        % Reload state
        X_k = [pos_n;vel_b;b;bias_acc;bias_gyr;bias_baro];

        %% Matrix F Elements with dx_dot = F*dx + G*w
        % F_vp, F_rp and w_ie neglected
        
        F_pv = R_b_n;
        
        F_vr = R_n_b*skew_matrix(g_n);
        
        F_vbf = -eye(3);
        
        F_vbg = -skew_matrix(vel_b);
        
        F_rbg = -R_b_n;
        
        F = [   zeros(3,3)	F_pv      	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(3,3)	zeros(3,3)	F_vr      	F_vbf       F_vbg     	zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	F_rbg       zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,3)	zeros(3,1);
                zeros(1,3) 	zeros(1,3)  zeros(1,3)  zeros(1,3)  zeros(1,3)  0];
        
        G = [
            zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,1);
            -eye(3)    F_vbg	  zeros(3,3) zeros(3,3) zeros(3,1);
            zeros(3,3) -R_b_n     zeros(3,3) zeros(3,3) zeros(3,1);
            zeros(3,3) zeros(3,3) eye(3,3)   zeros(3,3) zeros(3,1);
            zeros(3,3) zeros(3,3) zeros(3,3) eye(3,3)   zeros(3,1);
            zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,3) 1];

        %%%%%%%%%%%%%%%%%%%
        %% KALMAN FILTER %%
        %%%%%%%%%%%%%%%%%%%

        % Test if AUV is static
        if norm(w_ib_b) < CRIT_STATIC_GYRO && abs(norm(R_b_n*f_b) - norm(g_n)) < CRIT_STATIC_ACC

            counter_static = counter_static + 1;

        else

            counter_static = 0;

        end

        if counter_static*dt > TIME_STATIC
            isStatic = 1
        else
            isStatic = 0;
        end

        % Discretization
        G_k = G;
        
        Q = G_k*Qc*G_k'; % Equation (5.14) - Lavoie
        
        Q_k = Q*dt + (F*Q + Q*F')*(dt^2)/2;% 2nd order Taylor series(Lavoie - Page 94)
        
        Phi_k = eye(length(F)) + (dt*F)^1/factorial(1) + (dt*F)^2/factorial(2); 

        P = Phi_k*P*Phi_k' + Q_k;
        
        %%%%%%%%%%%%
        %% UPDATE %%
        %%%%%%%%%%%%

        if ( (imuHasNewData == ACTIVE(1)) || (dvlHasNewData == ACTIVE(2)) || (baroHasNewData == ACTIVE(3)) || (isStatic == ACTIVE(4)) ) % Apply correction when new measurements are available from GPS, MAG or BARO
            disp('update');
            % Set d_z to zero
            d_z_euler = zeros(3,1);
            d_z_dvl = zeros(3,1);
            d_z_baro = 0;
            d_z_zero = zeros(3,1);
            
            % Init large R
            R_EULER_k    = 1e5*ones(1,3);
            R_DVL_k    = 1e5*ones(1,3);
            R_BARO_k   = 1e5;
            R_ZERO_k   = 1e5*ones(1,3);
            
            % Init H matrix
            H_EULER_k = zeros(3,16);
            H_DVL_k = zeros(3,16);
            H_BARO_k = zeros(1,16);
            H_ZERO_k = zeros(3,16);
            
            if imuHasNewData == ACTIVE(1)
                
                quat_meas = block.InputPort(2).Data; % Extract measurements
                
                % Aiding Measurement Model
                [phi, theta, psi] = quat2euler(b);
                [phi_meas, theta_meas, psi_meas] = quat2euler(quat_meas);
                euler_meas = [phi_meas;theta_meas;psi_meas];
                
                skew_T = [cos(psi_meas)*cos(theta_meas)     -sin(psi_meas)	0;
                          sin(psi_meas)*cos(theta_meas)     cos(psi_meas)	0;
                          -sin(theta_meas)                  0               1];
                inv_skew_T = inv(skew_T);
                H_EULER = [zeros(3,3) zeros(3,3) inv_skew_T zeros(3,3) zeros(3,3) zeros(3,1)];
                H_EULER_k = H_EULER;
                
                euler_hat = [phi;theta;psi]; % Prediction
                
                d_z_euler = euler_meas - euler_hat;
                block.Dwork(3).Data(1:3) = d_z_euler;
                
                % Measurements covariance matrix
                R_EULER_k = [SIGMA_MEAS_IMU_ROLL SIGMA_MEAS_IMU_PITCH SIGMA_MEAS_IMU_YAW];
                
                imuHasNewData = 0;
            end
            
            if dvlHasNewData == ACTIVE(2)
                
                dvl_meas = block.InputPort(3).Data(1:3); % Extract measurements
                bottom_status = block.InputPort(3).Data(4);
                velocity_error = block.InputPort(3).Data(5);
                
                if bottom_status == 0
                    % Aiding Measurement Model
                    skew_l_pD = skew_matrix(l_pD);
                    H_DVL = [zeros(3,3) eye(3) zeros(3,3) zeros(3,3) skew_l_pD zeros(3,1)];
                    H_DVL_k = H_DVL;

                    dvl_hat = vel_b + cross(w_ib_b,l_pD); % Prediction

                    d_z_dvl = dvl_meas - dvl_hat;
                    block.Dwork(3).Data(4:6) = d_z_dvl;

                    % Measurements covariance matrix
                    R_DVL_k = (1+norm(velocity_error))*[SIGMA_MEAS_DVL_X SIGMA_MEAS_DVL_Y SIGMA_MEAS_DVL_Z];
                end
                
                dvlHasNewData = 0;
            end
            
            if baroHasNewData == ACTIVE(3)
                
                baro_meas = block.InputPort(4).Data - bias_baro;
                
                if baro_meas > 100000 % is underwater ? [Pa]
                    
                    depth_meas = depth_calc(baro_meas); % Extract measurements

                    % Aiding Measurement Model
                    l_tp = R_b_n * l_pp;
                    skew_l_tp = skew_matrix(l_tp);
                    H_BARO = [0 0 1 zeros(1,3) -[0 0 1]*skew_l_tp zeros(1,3) zeros(1,3) 1/(1000*ge)];
                    H_BARO_k = H_BARO;

                    depth_hat = pos_n(3); % Prediction

                    d_z_baro = depth_meas - depth_hat;
                    block.Dwork(3).Data(7) = d_z_baro;

                    % Measurements covariance matrix
                    R_BARO_k = SIGMA_MEAS_BARO;
                end
                
                baroHasNewData = 0;
            end
            
            if isStatic
                
                zero_meas = [0;0;0]; % Extract measurements
                
                % Aiding Measurement Model
                H_ZERO = [zeros(3,3) eye(3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,1)];
                H_ZERO_k = H_ZERO;
                
                zero_hat = vel_b; % Prediction
                
                d_z_zero = zero_meas - zero_hat;
                block.Dwork(3).Data(8:10) = d_z_zero;
                
                % Measurements covariance matrix
                R_ZERO_k = [SIGMA_MEAS_STATIC SIGMA_MEAS_STATIC SIGMA_MEAS_STATIC];
                
            end

            % Prediction stage
            H_KF_EULER_DVL_BARO_ZERO_k = [H_EULER_k; H_DVL_k; H_BARO_k;H_ZERO_k]; % H matrix for EKF
            
            % Measurements
            d_z_euler_dvl_baro_zero = [d_z_euler;d_z_dvl;d_z_baro;d_z_zero];
            
            % Measurements noise covariance
            R_EULER_DVL_BARO_ZERO_k = diag([R_EULER_k R_DVL_k R_BARO_k R_ZERO_k]);
            
            % Kalman gain calculation
            K_EULER_DVL_BARO_ZERO = P*H_KF_EULER_DVL_BARO_ZERO_k'/(H_KF_EULER_DVL_BARO_ZERO_k*P*H_KF_EULER_DVL_BARO_ZERO_k'+ R_EULER_DVL_BARO_ZERO_k);
            
            d_x_hat = K_EULER_DVL_BARO_ZERO*d_z_euler_dvl_baro_zero;
            
            % Update state covariance matrix using 'Joseph form'
            P = (eye(16) - K_EULER_DVL_BARO_ZERO*H_KF_EULER_DVL_BARO_ZERO_k)*...
                P*(eye(16) - K_EULER_DVL_BARO_ZERO*H_KF_EULER_DVL_BARO_ZERO_k)' +...
                K_EULER_DVL_BARO_ZERO*R_EULER_DVL_BARO_ZERO_k*K_EULER_DVL_BARO_ZERO';
            
            % State covariance matrix diagonalization
            P = (P + P')/2;
            
            block.Dwork(2).Data = d_x_hat;
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
    
    P = BARO;

    P0 = 100000; % Pressure init at surface [Pa]

    rho_water = 1000; % [kh/m3]

    depth = (BARO-P0) / ( rho_water * ge ); % Saunder-Fofonoff equation

end


