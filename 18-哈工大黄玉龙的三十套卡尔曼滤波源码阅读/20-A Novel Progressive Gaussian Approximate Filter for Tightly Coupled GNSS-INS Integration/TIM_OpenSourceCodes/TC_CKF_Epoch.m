function [est_C_b_e_new,est_v_eb_e_new,est_r_eb_e_new,est_IMU_bias_new,...
            est_clock_new,P_matrix_new] = TC_CKF_Epoch(GNSS_measurements,...
            no_meas,tor_s,est_C_b_e_old,est_v_eb_e_old,est_r_eb_e_old,...
            est_IMU_bias_old,est_clock_old,P_matrix_old,meas_f_ib_b,...
            est_L_b_old,Q_prime_matrix,R_matrix)
        
%TC_CKF_Epoch - Implements one cycle of the tightly coupled INS/GNSS
% cubture Kalman filter plus closed-loop correction of all inertial states
%
% Most of the code copy from "Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition", which is created 27/5/12 by Paul Groves

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

% Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
x_est_propagated(1:15,1) = 0;
x_est_propagated(16,1) = est_clock_old(1) + est_clock_old(2) * tor_s;
x_est_propagated(17,1) = est_clock_old(2);

% TIME UPTATE

nx=size(x_est_propagated,1);                                         
cub_point=2*nx;  
nPts=cub_point;
s_dim = nx;
W = 1/cub_point;
Kesi = sqrt(s_dim)*[eye(17), -eye(17)];
% Propagate state estimation and covariance matrix 
[U, s_hat, V] = svd(P_matrix_old);
s_hat = U*sqrt(s_hat);
x_cp = zeros(s_dim, cub_point);
x_minus = zeros(s_dim, cub_point);
x_hat = zeros(s_dim, 1);
for cp = 1 : cub_point
    x_cp(:, cp) = s_hat * Kesi(:, cp) + x_est_propagated;
    x_minus(:, cp) = Phi_matrix * x_cp(:, cp);
    x_hat = W * x_minus(:, cp) + x_hat;
end
P_matrix_propagated = zeros(s_dim, s_dim);
for cp = 1 : cub_point
    P_matrix_propagated = W*x_minus(:, cp)*x_minus(:, cp)'+P_matrix_propagated;
end
P_matrix_propagated = P_matrix_propagated-x_hat*x_hat'+Q_prime_matrix;

% Generate cubture point
[Xi]=genetate_CPt(x_hat,P_matrix_propagated);

% MEASUREMENT UPDATE PHASE

Zi_all_sta=[];
zkk1_all_sta=[];
real_z_all_sta=[];

for j = 1:no_meas
    
    Zi=ckf_Mst(Xi,GNSS_measurements,est_r_eb_e_old,est_v_eb_e_old,j);

    zkk1=sum(Zi,2)/nPts;
    
    real_range=GNSS_measurements(j,1);
    
    real_vel=GNSS_measurements(j,2);
    
    real_z=[real_range;real_vel];
    
    Zi_all_sta=[Zi_all_sta; Zi];
    
    zkk1_all_sta=[zkk1_all_sta;zkk1];
    
    real_z_all_sta=[real_z_all_sta; real_z];
    
end % for j   

delta_z = real_z_all_sta - zkk1_all_sta;

Pzzkk1 = Zi_all_sta*Zi_all_sta'/nPts-zkk1_all_sta*zkk1_all_sta'+R_matrix;

Pxzkk1 = Xi*Zi_all_sta'/nPts-x_hat*zkk1_all_sta';

K_matrix = Pxzkk1*inv(Pzzkk1);            
    
x_est_new = x_est_propagated + K_matrix * delta_z;

P_matrix_new = P_matrix_propagated-K_matrix*Pzzkk1*K_matrix' + Q_prime_matrix; 

% CLOSED-LOOP CORRECTION

% Correct attitude, velocity, and position using (14.7-9)
est_C_b_e_new = (eye(3) - Skew_symmetric(x_est_new(1:3))) * est_C_b_e_old;
est_v_eb_e_new = est_v_eb_e_old - x_est_new(4:6);
est_r_eb_e_new = est_r_eb_e_old - x_est_new(7:9);

% Update IMU bias and GNSS receiver clock estimates
est_IMU_bias_new = est_IMU_bias_old + x_est_new(10:15);
est_clock_new = x_est_new(16:17)';

% Ends