function [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
        est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n)
%Calculate_errors_NED - Calculates the position, velocity, and attitude
% errors of a NED navigation solution.
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 3/4/2012 by Paul Groves
%
% Inputs:
%   est_L_b       latitude solution (rad)
%   est_lambda_b  longitude solution (rad)
%   est_h_b       height solution (m)
%   est_v_eb_n    velocity solution of body frame w.r.t. ECEF frame,
%                 resolved along north, east, and down (m/s)
%   est_C_b_n     body-to-NED coordinate transformation matrix solution
%   true_L_b      true latitude (rad)
%   true_lambda_b true longitude (rad)
%   true_h_b      true height (m)
%   true_v_eb_n   true velocity of body frame w.r.t. ECEF frame, resolved
%                 along north, east, and down (m/s)
%   C_b_n         true body-to-NED coordinate transformation matrix
%
% Outputs:
%   delta_r_eb_n     position error resolved along NED (m)
%   delta_v_eb_n     velocity error resolved along NED (m/s)
%   delta_eul_nb_n   attitude error as NED Euler angles (rad)
%                    These are expressed about north, east, and down

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Position error calculation, using (2.119)
[R_N,R_E] = Radii_of_curvature(true_L_b);
delta_r_eb_n(1) = (est_L_b - true_L_b) * (R_N + true_h_b);
delta_r_eb_n(2) = (est_lambda_b - true_lambda_b) * (R_E + true_h_b) *...
    cos(true_L_b);
delta_r_eb_n(3) = -(est_h_b - true_h_b);

% Velocity error calculation
delta_v_eb_n = est_v_eb_n - true_v_eb_n;

% Attitude error calculation, using (5.109) and (5.111)
delta_C_b_n = est_C_b_n * true_C_b_n';
delta_eul_nb_n = -CTM_to_Euler(delta_C_b_n);
    
% Ends