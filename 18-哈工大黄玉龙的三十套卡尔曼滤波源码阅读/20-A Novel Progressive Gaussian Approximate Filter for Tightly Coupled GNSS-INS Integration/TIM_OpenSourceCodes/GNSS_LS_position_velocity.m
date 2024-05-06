function [est_r_ea_e,est_v_ea_e,est_clock] = GNSS_LS_position_velocity(...
    GNSS_measurements,no_GNSS_meas,predicted_r_ea_e,predicted_v_ea_e)
%GNSS_LS_position_velocity - Calculates position, velocity, clock offset, 
%and clock drift using unweighted iterated least squares. Separate
%calculations are implemented for position and clock offset and for
%velocity and clock drift
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%   no_GNSS_meas          Number of satellites for which measurements are
%                         supplied
%   predicted_r_ea_e      prior predicted ECEF user position (m)
%   predicted_v_ea_e      prior predicted ECEF user velocity (m/s)
%
% Outputs:
%   est_r_ea_e            estimated ECEF user position (m)
%   est_v_ea_e            estimated ECEF user velocity (m/s)
%   est_clock             estimated receiver clock offset (m) and drift (m/s)
 
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s

% Begins

% POSITION AND CLOCK OFFSET

% Setup predicted state
x_pred(1:3,1) = predicted_r_ea_e;
x_pred(4,1) = 0;
test_convergence = 1;

% Repeat until convergence
while test_convergence>0.0001
    
    % Loop measurements
    for j = 1:no_GNSS_meas

        % Predict approx range 
        delta_r = GNSS_measurements(j,3:5)' - x_pred(1:3);
        approx_range = sqrt(delta_r' * delta_r);

        % Calculate frame rotation during signal transit time using (8.36)
        C_e_I = [1, omega_ie * approx_range / c, 0;...
                 -omega_ie * approx_range / c, 1, 0;...
                 0, 0, 1];

        % Predict pseudo-range using (9.143)
        delta_r = C_e_I *  GNSS_measurements(j,3:5)' - x_pred(1:3);
        range = sqrt(delta_r' * delta_r);
        pred_meas(j,1) = range + x_pred(4);
        
        % Predict line of sight and deploy in measurement matrix, (9.144)
        H_matrix (j,1:3) = - delta_r' / range;
        H_matrix (j,4) = 1;
        
    end % for j
        
    % Unweighted least-squares solution, (9.35)/(9.141)
    x_est = x_pred + inv(H_matrix(1:no_GNSS_meas,:)' *...
        H_matrix(1:no_GNSS_meas,:)) * H_matrix(1:no_GNSS_meas,:)' *...
        (GNSS_measurements(1:no_GNSS_meas,1) -  pred_meas(1:no_GNSS_meas));

    % Test convergence    
    test_convergence = sqrt((x_est - x_pred)' * (x_est - x_pred));
    
    % Set predictions to estimates for next iteration
    x_pred = x_est;
    
end % while

% Set outputs to estimates
est_r_ea_e(1:3,1) = x_est(1:3);
est_clock(1) = x_est(4);

% VELOCITY AND CLOCK DRIFT

% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);
       
% Setup predicted state
x_pred(1:3,1) = predicted_v_ea_e;
x_pred(4,1) = 0;
test_convergence = 1;

% Repeat until convergence
while test_convergence>0.0001
    
    % Loop measurements
    for j = 1:no_GNSS_meas

        % Predict approx range 
        delta_r = GNSS_measurements(j,3:5)' - est_r_ea_e;
        approx_range = sqrt(delta_r' * delta_r);

        % Calculate frame rotation during signal transit time using (8.36)
        C_e_I = [1, omega_ie * approx_range / c, 0;...
                 -omega_ie * approx_range / c, 1, 0;...
                 0, 0, 1];

        % Calculate range using (8.35)
        delta_r = C_e_I *  GNSS_measurements(j,3:5)' - est_r_ea_e;
        range = sqrt(delta_r' * delta_r);

        % Calculate line of sight using (8.41)
        u_as_e = delta_r / range;
        
        % Predict pseudo-range rate using (9.143)
        range_rate = u_as_e' * (C_e_I * (GNSS_measurements(j,6:8)' +...
            Omega_ie * GNSS_measurements(j,3:5)') - (x_pred(1:3) +...
            Omega_ie * est_r_ea_e));        
        pred_meas(j,1) = range_rate + x_pred(4);
        
        % Predict line of sight and deploy in measurement matrix, (9.144)
        H_matrix (j,1:3) = - u_as_e';
        H_matrix (j,4) = 1;
        
    end % for j
    
    % Unweighted least-squares solution, (9.35)/(9.141)
    x_est = x_pred + inv(H_matrix(1:no_GNSS_meas,:)' *...
        H_matrix(1:no_GNSS_meas,:)) * H_matrix(1:no_GNSS_meas,:)' *...
        (GNSS_measurements(1:no_GNSS_meas,2) -  pred_meas(1:no_GNSS_meas));

    % Test convergence    
    test_convergence = sqrt((x_est - x_pred)' * (x_est - x_pred));
    
    % Set predictions to estimates for next iteration
    x_pred = x_est;
    
end % while

% Set outputs to estimates
est_v_ea_e(1:3,1) = x_est(1:3);
est_clock(2) = x_est(4);

% Ends