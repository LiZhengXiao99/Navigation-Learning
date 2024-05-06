function g = Gravity_ECEF(r_eb_e)
%Gravitation_ECI - Calculates  acceleration due to gravity resolved about 
%ECEF-frame
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   r_eb_e  Cartesian position of body frame w.r.t. ECEF frame, resolved
%           about ECEF-frame axes (m)
% Outputs:
%   g       Acceleration due to gravity (m/s^2)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

%Parameters
R_0 = 6378137; %WGS84 Equatorial radius in meters
mu = 3.986004418E14; %WGS84 Earth gravitational constant (m^3 s^-2)
J_2 = 1.082627E-3; %WGS84 Earth's second gravitational constant
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

% Calculate distance from center of the Earth
mag_r = sqrt(r_eb_e' * r_eb_e);

% If the input position is 0,0,0, produce a dummy output
if mag_r==0
    g = [0;0;0];
    
% Calculate gravitational acceleration using (2.142)
else
    z_scale = 5 * (r_eb_e(3) / mag_r)^2;
    gamma = -mu / mag_r^3 *(r_eb_e + 1.5 * J_2 * (R_0 / mag_r)^2 *...
        [(1 - z_scale) * r_eb_e(1); (1 - z_scale) * r_eb_e(2);...
        (3 - z_scale) * r_eb_e(3)]);

    % Add centripetal acceleration using (2.133)
    g(1:2,1) = gamma(1:2) + omega_ie^2 * r_eb_e(1:2);
    g(3) = gamma(3);
    
end % if

% Ends