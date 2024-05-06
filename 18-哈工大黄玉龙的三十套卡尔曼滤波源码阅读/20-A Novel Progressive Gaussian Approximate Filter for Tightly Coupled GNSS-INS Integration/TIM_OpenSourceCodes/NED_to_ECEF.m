function [r_eb_e,v_eb_e,C_b_e] = NED_to_ECEF(L_b,lambda_b,h_b,v_eb_n,C_b_n)
%NED_to_ECEF - Converts curvilinear to Cartesian position, velocity
%resolving axes from NED to ECEF and attitude from NED- to ECEF-referenced
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs:
%   L_b           latitude (rad)
%   lambda_b      longitude (rad)
%   h_b           height (m)
%   v_eb_n        velocity of body frame w.r.t. ECEF frame, resolved along
%                 north, east, and down (m/s)
%   C_b_n         body-to-NED coordinate transformation matrix
%
% Outputs:
%   r_eb_e        Cartesian position of body frame w.r.t. ECEF frame, resolved
%                 along ECEF-frame axes (m)
%   v_eb_e        velocity of body frame w.r.t. ECEF frame, resolved along
%                 ECEF-frame axes (m/s)
%   C_b_e         body-to-ECEF-frame coordinate transformation matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Parameters
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Begins

% Calculate transverse radius of curvature using (2.105)
R_E = R_0 / sqrt(1 - (e * sin(L_b))^2);

% Convert position using (2.112)
cos_lat = cos(L_b);
sin_lat = sin(L_b);
cos_long = cos(lambda_b);
sin_long = sin(lambda_b);
r_eb_e = [(R_E + h_b) * cos_lat * cos_long;...
          (R_E + h_b) * cos_lat * sin_long;...
          ((1 - e^2) * R_E + h_b) * sin_lat];
      
% Calculate ECEF to NED coordinate transformation matrix using (2.150)
C_e_n = [-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat;...
                   -sin_long,            cos_long,        0;...
         -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat];
     
% Transform velocity using (2.73)
v_eb_e = C_e_n' * v_eb_n;

% Transform attitude using (2.15)
C_b_e = C_e_n' * C_b_n;

% Ends