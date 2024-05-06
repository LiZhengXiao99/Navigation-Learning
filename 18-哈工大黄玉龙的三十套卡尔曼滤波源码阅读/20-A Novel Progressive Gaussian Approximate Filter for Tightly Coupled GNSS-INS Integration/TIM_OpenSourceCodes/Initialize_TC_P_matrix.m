function P_matrix = Initialize_TC_P_matrix(TC_KF_config)
%Initialize_TC_P_matrix - Initializes the tightly coupled INS/GNSS EKF
%error covariance matrix
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   TC_KF_config
%     .init_att_unc           Initial attitude uncertainty per axis (rad)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_b_a_unc           Initial accel. bias uncertainty (m/s^2)
%     .init_b_g_unc           Initial gyro. bias uncertainty (rad/s)
%     .init_clock_offset_unc  Initial clock offset uncertainty per axis (m)
%     .init_clock_drift_unc   Initial clock drift uncertainty per axis (m/s)
%
% Outputs:
%   P_matrix              state estimation error covariance matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Initialize error covariance matrix
P_matrix =  zeros(17);
P_matrix(1:3,1:3) = eye(3) * TC_KF_config.init_att_unc^2;
P_matrix(4:6,4:6) = eye(3) * TC_KF_config.init_vel_unc^2;
P_matrix(7:9,7:9) = eye(3) * TC_KF_config.init_pos_unc^2;
P_matrix(10:12,10:12) = eye(3) * TC_KF_config.init_b_a_unc^2;
P_matrix(13:15,13:15) = eye(3) * TC_KF_config.init_b_g_unc^2;
P_matrix(16,16) = TC_KF_config.init_clock_offset_unc^2;
P_matrix(17,17) = TC_KF_config.init_clock_drift_unc^2;

% Ends