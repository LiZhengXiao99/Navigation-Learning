function est_C_b_n = Initialize_NED_attitude(C_b_n,initialization_errors)
%Initialize_NED_attitude - Initializes the attitude solution by adding 
%errors to the truth.
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   C_b_n         true body-to-NED coordinate transformation matrix
%   initialization_errors
%     .delta_eul_nb_n   attitude error as NED Euler angles (rad)
%
% Outputs:
%   est_C_b_n     body-to-NED coordinate transformation matrix solution

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Attitude initialization, using (5.109) and (5.111)
delta_C_b_n = Euler_to_CTM(-initialization_errors.delta_eul_nb_n);
est_C_b_n = delta_C_b_n * C_b_n;
    
% Ends