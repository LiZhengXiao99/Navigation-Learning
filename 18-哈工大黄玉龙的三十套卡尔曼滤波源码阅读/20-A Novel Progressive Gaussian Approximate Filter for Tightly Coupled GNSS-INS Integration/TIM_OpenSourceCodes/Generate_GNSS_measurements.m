function [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(...
    time, sat_r_es_e,sat_v_es_e,r_ea_e,L_a,lambda_a,v_ea_e,...
    GNSS_biases,GNSS_config)
%Generate_GNSS_measurements - Generates a set of pseudo-range and pseudo-
%range rate measurements for all satellites above the elevation mask angle 
%and adds satellite positions and velocities to the datesets. 
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   time                    Current simulation time
%   sat_r_es_e (no_sat x 3) ECEF satellite positions (m)
%   sat_v_es_e (no_sat x 3) ECEF satellite velocities (m/s)
%   r_ea_e                  ECEF user position (m)
%   L_a                     user latitude (rad)
%   lambda_a                user longitude (rad)
%   v_ea_e                  ECEF user velocity (m/s)
%   GNSS_biases (no_sat)    Bias-like GNSS range errors (m)
%   GNSS_config
%     .no_sat             Number of satellites in constellation
%     .mask_angle         Mask angle (deg)
%     .code_track_err_SD  Code tracking error SD (m)
%     .rate_track_err_SD  Range rate tracking error SD (m/s)
%     .rx_clock_offset    Receiver clock offset at time=0 (m)
%     .rx_clock_drift     Receiver clock drift at time=0 (m/s)
%
% Outputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%   no_GNSS_meas          Number of satellites for which measurements are
%                         supplied

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s

% Begins
no_GNSS_meas = 0;

% Calculate ECEF to NED coordinate transformation matrix using (2.150)
cos_lat = cos(L_a);
sin_lat = sin(L_a);
cos_long = cos(lambda_a);
sin_long = sin(lambda_a);
C_e_n = [-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat;...
                   -sin_long,            cos_long,        0;...
         -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat];
     
% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
       
% Loop satellites
for j = 1:GNSS_config.no_sat

    % Determine ECEF line-of-sight vector using (8.41)
    delta_r = sat_r_es_e(j,1:3)' - r_ea_e;
    approx_range = sqrt(delta_r' * delta_r);
    u_as_e = delta_r / approx_range;
    
    % Convert line-of-sight vector to NED using (8.39) and determine
    % elevation using (8.57)
    elevation = -asin(C_e_n(3,:) * u_as_e);
    
    % Determine if satellite is above the masking angle
    if (elevation >= degtorad(GNSS_config.mask_angle)); % && no_GNSS_meas<4;
    
        % Increment number of measurements
        no_GNSS_meas = no_GNSS_meas + 1;
    
        % Calculate frame rotation during signal transit time using (8.36)
        C_e_I = [1, omega_ie * approx_range / c, 0;...
                 -omega_ie * approx_range / c, 1, 0;...
                 0, 0, 1];

        % Calculate range using (8.35)
        delta_r = C_e_I * sat_r_es_e(j,1:3)' - r_ea_e;
        range = sqrt(delta_r' * delta_r);
        
        % Calculate range rate using (8.44)
         range_rate = u_as_e' * (C_e_I * (sat_v_es_e(j,1:3)' + Omega_ie *...
            sat_r_es_e(j,1:3)') - (v_ea_e + Omega_ie * r_ea_e));
    
        % Calculate pseudo-range measurement
        GNSS_measurements(no_GNSS_meas,1) = range + GNSS_biases(j) +...
            GNSS_config.rx_clock_offset + GNSS_config.rx_clock_drift *...
            time + GNSS_config.code_track_err_SD * randn;
    
        % Calculate pseudo-range rate measurement
        GNSS_measurements(no_GNSS_meas,2) = range_rate +...
            GNSS_config.rx_clock_drift + GNSS_config.rate_track_err_SD *...
            randn;
    
        % Append satellite position and velocity to output data
        GNSS_measurements(no_GNSS_meas,3:5) = sat_r_es_e(j,1:3);
        GNSS_measurements(no_GNSS_meas,6:8) = sat_v_es_e(j,1:3);
    
    end % elevation     
end % for j

% Ends