function [r_eb_e,v_eb_e,C_b_e] = Nav_equations_ECEF(tor_i,old_r_eb_e,...
        old_v_eb_e,old_C_b_e,f_ib_b,omega_ib_b)
%Nav_equations_ECEF - Runs precision ECEF-frame inertial navigation
%equations
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   tor_i         time interval between epochs (s)
%   old_r_eb_e    previous Cartesian position of body frame w.r.t. ECEF
%                 frame, resolved along ECEF-frame axes (m)
%   old_C_b_e     previous body-to-ECEF-frame coordinate transformation matrix
%   old_v_eb_e    previous velocity of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m/s)
%   f_ib_b        specific force of body frame w.r.t. ECEF frame, resolved
%                 along body-frame axes, averaged over time interval (m/s^2)
%   omega_ib_b    angular rate of body frame w.r.t. ECEF frame, resolved
%                 about body-frame axes, averaged over time interval (rad/s)
% Outputs:
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% parameters
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

% ATTITUDE UPDATE
% From (2.145) determine the Earth rotation over the update interval
% C_Earth = C_e_i' * old_C_e_i
alpha_ie = omega_ie * tor_i;
C_Earth = [cos(alpha_ie), sin(alpha_ie), 0;...
          -sin(alpha_ie), cos(alpha_ie), 0;...
                       0,             0,  1];
                       
% Calculate attitude increment, magnitude, and skew-symmetric matrix
alpha_ib_b = omega_ib_b * tor_i;
mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
Alpha_ib_b = Skew_symmetric(alpha_ib_b);  

% Obtain coordinate transformation matrix from the new attitude w.r.t. an
% inertial frame to the old using Rodrigues' formula, (5.73)
if mag_alpha>1.E-8
    C_new_old = eye(3) + sin(mag_alpha) / mag_alpha * Alpha_ib_b +...
        (1 - cos(mag_alpha)) / mag_alpha^2 * Alpha_ib_b * Alpha_ib_b;
else
    C_new_old = eye(3) + Alpha_ib_b;
end %if mag_alpha    

% Update attitude using (5.75)
C_b_e = C_Earth * old_C_b_e * C_new_old;
    
% SPECIFIC FORCE FRAME TRANSFORMATION
% Calculate the average body-to-ECEF-frame coordinate transformation
% matrix over the update interval using (5.84) and (5.85)
if mag_alpha>1.E-8
    ave_C_b_e = old_C_b_e * (eye(3) + (1 - cos(mag_alpha)) / mag_alpha^2 ...
        * Alpha_ib_b + (1 - sin(mag_alpha) / mag_alpha) / mag_alpha^2 ...
        * Alpha_ib_b * Alpha_ib_b) - 0.5 * Skew_symmetric([0;0;alpha_ie])...
        * old_C_b_e;
else
     ave_C_b_e = old_C_b_e - 0.5 * Skew_symmetric([0;0;alpha_ie]) *...
         old_C_b_e;
end %if mag_alpha     

% Transform specific force to ECEF-frame resolving axes using (5.85)
f_ib_e = ave_C_b_e * f_ib_b;
    
% UPDATE VELOCITY
% From (5.36),
v_eb_e = old_v_eb_e + tor_i * (f_ib_e + Gravity_ECEF(old_r_eb_e) -...
    2 * Skew_symmetric([0;0;omega_ie]) * old_v_eb_e);

% UPDATE CARTESIAN POSITION
% From (5.38),
r_eb_e = old_r_eb_e + (v_eb_e + old_v_eb_e) * 0.5 * tor_i; 

% Ends