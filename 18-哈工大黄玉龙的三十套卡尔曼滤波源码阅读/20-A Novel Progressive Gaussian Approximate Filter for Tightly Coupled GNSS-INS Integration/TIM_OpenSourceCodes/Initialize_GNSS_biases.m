function GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,r_ea_e,L_a,...
    lambda_a,GNSS_config)
%Initialize_GNSS_biases - Initializes the GNSS range errors due to signal
%in space, ionosphere and troposphere errors based on the elevation angles. 
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 11/4/2012 by Paul Groves
%
% Inputs:
%   sat_r_es_e (no_sat x 3) ECEF satellite positions (m)
%   r_ea_e                ECEF user position (m)
%   L_a                   user latitude (rad)
%   lambda_a              user longitude (rad)
%   GNSS_config
%     .no_sat             Number of satellites in constellation
%     .mask_angle         Mask angle (deg)
%     .SIS_err_SD         Signal in space error SD (m)
%     .zenith_iono_err_SD Zenith ionosphere error SD (m)
%     .zenith_trop_err_SD Zenith troposphere error SD (m)
%
% Outputs:
%   GNSS_biases (no_sat)  Bias-like GNSS range errors

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Calculate ECEF to NED coordinate transformation matrix using (2.150)
cos_lat = cos(L_a);
sin_lat = sin(L_a);
cos_long = cos(lambda_a);
sin_long = sin(lambda_a);
C_e_n = [-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat;...
                   -sin_long,            cos_long,        0;...
         -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat];
     
% Loop satellites
for j = 1:GNSS_config.no_sat

    % Determine ECEF line-of-sight vector using (8.41)
    delta_r = sat_r_es_e(j,1:3)' - r_ea_e;
    u_as_e = delta_r / sqrt(delta_r' * delta_r);
    
    % Convert line-of-sight vector to NED using (8.39) and determine
    % elevation using (8.57)
    elevation = -asin(C_e_n(3,:) * u_as_e);
    
    % Limit the minimum elevation angle to the masking angle
    elevation = max(elevation,degtorad(GNSS_config.mask_angle));
    
    % Calculate ionosphere and troposphere error SDs using (9.79) and (9.80)
    iono_SD = GNSS_config.zenith_iono_err_SD / sqrt(1 - 0.899 *...
        cos(elevation)^2);
    trop_SD = GNSS_config.zenith_trop_err_SD / sqrt(1 - 0.998 *...
        cos(elevation)^2);
    
    % Determine range bias
    GNSS_biases(j) = GNSS_config.SIS_err_SD*randn + iono_SD*randn +...
        trop_SD*randn;
         
end % for j

% Ends