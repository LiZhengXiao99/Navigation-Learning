function [R_N,R_E]= Radii_of_curvature(L)
%Radii_of_curvature - Calculates the meridian and transverse radii of
%curvature
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 31/3/2012 by Paul Groves
%
% Inputs:
%   L   geodetic latitude (rad)
%
% Outputs:
%   R_N   meridian radius of curvature (m)
%   R_E   transverse radius of curvature (m)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Parameters
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Calculate meridian radius of curvature using (2.105)
temp = 1 - (e * sin(L))^2; 
R_N = R_0 * (1 - e^2) / temp^1.5;

% Calculate transverse radius of curvature using (2.105)
R_E = R_0 / sqrt(temp);

% Ends