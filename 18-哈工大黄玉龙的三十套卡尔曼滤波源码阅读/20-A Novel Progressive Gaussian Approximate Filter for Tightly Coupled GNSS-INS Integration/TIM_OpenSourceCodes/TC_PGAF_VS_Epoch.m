function [est_C_b_e_new,est_v_eb_e_new,est_r_eb_e_new,est_IMU_bias_new,...
            est_clock_new,P_matrix_new,alphaR,betaR] = TC_PGAF_VS_Epoch(GNSS_measurements,...
            no_meas,tor_s,est_C_b_e_old,est_v_eb_e_old,est_r_eb_e_old,...
            est_IMU_bias_old,est_clock_old,P_matrix_old,meas_f_ib_b,...
            est_L_b_old,Q_prime_matrix,R_matrix,alphaR_last,betaR_last)

%TC_PGAF_VS_Epoch - Implements one cycle of the tightly coupled INS/GNSS
% The proposed PGAF-VS Kalman filter plus closed-loop correction of all inertial states
%
% Most of the code copy from "Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition", which is created 27/5/12 by Paul Groves
%
% Inputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%   no_meas               Number of satellites for which measurements are
%                         supplied
%   tor_s                 propagation interval (s)
%   est_C_b_e_old         prior estimated body to ECEF coordinate
%                         transformation matrix
%   est_v_eb_e_old        prior estimated ECEF user velocity (m/s)
%   est_r_eb_e_old        prior estimated ECEF user position (m)
%   est_IMU_bias_old      prior estimated IMU biases (body axes)
%   est_clock_old         prior Kalman filter state estimates
%   P_matrix_old          previous Kalman filter error covariance matrix
%   meas_f_ib_b           measured specific force
%   est_L_b_old           previous latitude solution
%   TC_KF_config
%     .gyro_noise_PSD     Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD    Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD     Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD      Gyro bias random walk PSD (rad^2 s^-3)
%     .clock_freq_PSD     Receiver clock frequency-drift PSD (m^2/s^3)
%     .clock_phase_PSD    Receiver clock phase-drift PSD (m^2/s)
%     .pseudo_range_SD    Pseudo-range measurement noise SD (m)
%     .range_rate_SD      Pseudo-range rate measurement noise SD (m/s)
%
% Outputs:
%   est_C_b_e_new     updated estimated body to ECEF coordinate 
%                      transformation matrix
%   est_v_eb_e_new    updated estimated ECEF user velocity (m/s)
%   est_r_eb_e_new    updated estimated ECEF user position (m)
%   est_IMU_bias_new  updated estimated IMU biases
%     Rows 1-3          estimated accelerometer biases (m/s^2) 
%     Rows 4-6          estimated gyro biases (rad/s)
%   est_clock_new     updated Kalman filter state estimates
%     Row 1             estimated receiver clock offset (m) 
%     Row 2             estimated receiver clock drift (m/s)
%   P_matrix_new      updated Kalman filter error covariance matrix

 
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Begins

% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
       
% SYSTEM PROPAGATION PHASE

% Determine transition matrix using (14.50) (first-order approx)
Phi_matrix = eye(17);
Phi_matrix(1:3,1:3) = Phi_matrix(1:3,1:3) - Omega_ie * tor_s;
Phi_matrix(1:3,13:15) = est_C_b_e_old * tor_s;
Phi_matrix(4:6,1:3) = -tor_s * Skew_symmetric(est_C_b_e_old * meas_f_ib_b);
Phi_matrix(4:6,4:6) = Phi_matrix(4:6,4:6) - 2 * Omega_ie * tor_s;
geocentric_radius = R_0 / sqrt(1 - (e * sin(est_L_b_old))^2) *...
    sqrt(cos(est_L_b_old)^2 + (1 - e^2)^2 * sin(est_L_b_old)^2); % from (2.137)
Phi_matrix(4:6,7:9) = -tor_s * 2 * Gravity_ECEF(est_r_eb_e_old) /...
    geocentric_radius * est_r_eb_e_old' / sqrt (est_r_eb_e_old' *...
    est_r_eb_e_old);
Phi_matrix(4:6,10:12) = est_C_b_e_old * tor_s;
Phi_matrix(7:9,4:6) = eye(3) * tor_s;
Phi_matrix(16,17) = tor_s;

% TIME UPDATE PHASE

% Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
x_est_propagated(1:15,1) = 0;
x_est_propagated(16,1) = est_clock_old(1) + est_clock_old(2) * tor_s;
x_est_propagated(17,1) = est_clock_old(2);

% Propagate state estimation error covariance matrix using (3.46)
P_matrix_propagated = Phi_matrix * (P_matrix_old) *...  
    Phi_matrix' + 0.5 * Q_prime_matrix;

nx=size(x_est_propagated,1); 
nz=2*no_meas;
nPts=2*nx;
N=15; %recursive times  15
Nvb=20; %vb iterations times
sumri=0;
COUNTN=0;
rou = 1-exp(-4);

x_est_new=x_est_propagated;
P_matrix_new=P_matrix_propagated;

% MEASUREMENT UPDATE PHASE

for i=1:N
    COUNTN = COUNTN +1;
    %%%%%%%
    for j=1:2*no_meas
        alphaR(j)=rou*alphaR_last(j)+ 0.5;% + 0.5;
        betaR(j)=rou*betaR_last(j);
    end	
    E_R=R_matrix;

    diagR=[];
    
    %%%%%%%
    Eri_last=100;
    E_R_last=R_matrix;
    xkkvb_last=x_est_propagated;
    dist=1;
    count=0;
    
    while (dist>=1e-20)&&(count<=Nvb)
        
        %%%%%%%%update varitional step size
        [Zivb,real_z_all_sta]=ckf_MeasureUpdate_VB(x_est_new,P_matrix_new,...
                                     GNSS_measurements,est_r_eb_e_old,...
                                     est_v_eb_e_old,no_meas);
                                 
        Dk_j=(repmat(real_z_all_sta,1,nPts)-Zivb)*(repmat(real_z_all_sta,1,nPts)-Zivb)'/nPts;

        alfa=0.5*nz+1;
        
        beta=0.5*trace(Dk_j*inv(E_R));
        
        %%%%%%%%calculate expection of the varitional step size
        Eri=Calculate_Eri(sumri,alfa,beta);
        
        %%%%%%%%update R 
		for j=1:2*no_meas
			alphaR(j)=alphaR(j) + 0.5;
			betaR(j)=betaR(j) + 0.5*Dk_j(j,j);
            diagR=[diagR betaR(j)/alphaR(j)];
        end	
	
        %%%%%%%%calculate expection of R
        E_R=diag(diagR);
        diagR=[];
        
        %%%%%%%%update state     
        [x_est_new,P_matrix_new]=ckf_MeasureUpdate(x_est_propagated,P_matrix_propagated,... 
                                     GNSS_measurements,est_r_eb_e_old,...
                                     est_v_eb_e_old,no_meas,E_R/Eri);
        
        %%%%%%%%justify the terminal condition
        dist=(norm(Eri-Eri_last)/norm(Eri)+norm(E_R-E_R_last)/norm(E_R)+norm(x_est_new-xkkvb_last)/norm(x_est_new))/3;
        
        Eri_last=Eri;
        
        E_R_last=E_R;
        
        xkkvb_last=x_est_new;
        
        count=count+1;
    
    end % for while
    
    %%%%%%%%Initial value fore next iteration
    x_est_propagated=x_est_new; 
    P_matrix_propagated=P_matrix_new;
    
    %%%%%%%% accumulate the variational step size
    sumri=sumri+Eri;
    
    if (1-sumri)<0.01
        break;
    end
    
end % for i

%%%%%%%%absorb the remaining measurement information
rr=1-sumri;

if rr~=0
    [x_est_new,P_matrix_new]=ckf_MeasureUpdate(x_est_propagated,P_matrix_propagated,...
                             GNSS_measurements,est_r_eb_e_old,...
                             est_v_eb_e_old,no_meas,R_matrix/rr);
end

% CLOSED-LOOP CORRECTION

% Correct attitude, velocity, and position using (14.7-9)
est_C_b_e_new = (eye(3) - Skew_symmetric(x_est_new(1:3))) * est_C_b_e_old;
est_v_eb_e_new = est_v_eb_e_old - x_est_new(4:6);
est_r_eb_e_new = est_r_eb_e_old - x_est_new(7:9);

% Update IMU bias and GNSS receiver clock estimates
est_IMU_bias_new = est_IMU_bias_old + x_est_new(10:15);
est_clock_new = x_est_new(16:17)';

% Ends