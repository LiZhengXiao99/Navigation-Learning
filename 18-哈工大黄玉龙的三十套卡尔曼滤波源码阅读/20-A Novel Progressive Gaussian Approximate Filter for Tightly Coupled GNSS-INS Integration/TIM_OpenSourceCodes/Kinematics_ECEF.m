function [f_ib_b,omega_ib_b] = Kinematics_ECEF(tor_i,C_b_e,...
        old_C_b_e,v_eb_e,old_v_eb_e,r_eb_e)
%Kinematics_ECEF - calculates specific force and angular rate from input
%w.r.t and resolved along ECEF-frame axes
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix
%   old_C_b_e     previous body-to-ECEF-frame coordinate transformation matrix
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   old_v_eb_e    previous velocity of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m/s)
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
% Outputs:
%   f_ib_b        specific force of body frame w.r.t. ECEF frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECEF frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Parameters
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

if tor_i > 0

    % From (2.145) determine the Earth rotation over the update interval
    % C_Earth = C_e_i' * old_C_e_i
    alpha_ie = omega_ie * tor_i;
    C_Earth = [cos(alpha_ie), sin(alpha_ie), 0;...
              -sin(alpha_ie), cos(alpha_ie), 0;...
                           0,             0,  1];
    
    % Obtain coordinate transformation matrix from the old attitude (w.r.t.
    % an inertial frame) to the new
    C_old_new = C_b_e' * C_Earth * old_C_b_e;

    % Calculate the approximate angular rate w.r.t. an inertial frame
    alpha_ib_b(1,1) = 0.5 * (C_old_new(2,3) - C_old_new(3,2));
    alpha_ib_b(2,1) = 0.5 * (C_old_new(3,1) - C_old_new(1,3));
    alpha_ib_b(3,1) = 0.5 * (C_old_new(1,2) - C_old_new(2,1));

    % Calculate and apply the scaling factor
    temp = acos(0.5 * (C_old_new(1,1) + C_old_new(2,2) + C_old_new(3,3)...
        - 1.0));
    if temp>2e-5 %scaling is 1 if temp is less than this
        alpha_ib_b = alpha_ib_b * temp/sin(temp);
    end %if temp
    
    % Calculate the angular rate
    omega_ib_b = alpha_ib_b / tor_i;
    
    % Calculate the specific force resolved about ECEF-frame axes
    % From (5.36),
    f_ib_e = ((v_eb_e - old_v_eb_e) / tor_i) - Gravity_ECEF(r_eb_e)...
        + 2 * Skew_symmetric([0;0;omega_ie]) * old_v_eb_e;
    
    % Calculate the average body-to-ECEF-frame coordinate transformation
    % matrix over the update interval using (5.84) and (5.85)
    mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
    Alpha_ib_b = Skew_symmetric(alpha_ib_b);    
    if mag_alpha>1.E-8
        ave_C_b_e = old_C_b_e * (eye(3) + (1 - cos(mag_alpha)) /mag_alpha^2 ...
            * Alpha_ib_b + (1 - sin(mag_alpha) / mag_alpha) / mag_alpha^2 ...
            * Alpha_ib_b * Alpha_ib_b) - 0.5 * ...
            Skew_symmetric([0;0;alpha_ie]) * old_C_b_e;
    else
        ave_C_b_e = old_C_b_e -...
            0.5 * Skew_symmetric([0;0;alpha_ie]) * old_C_b_e;
    end %if mag_alpha
    
    % Transform specific force to body-frame resolving axes using (5.81)
    f_ib_b = inv(ave_C_b_e) * f_ib_e;
    
else
    % If time interval is zero, set angular rate and specific force to zero
    omega_ib_b = [0;0;0];
    f_ib_b = [0;0;0];
end %if tor_i
% Ends