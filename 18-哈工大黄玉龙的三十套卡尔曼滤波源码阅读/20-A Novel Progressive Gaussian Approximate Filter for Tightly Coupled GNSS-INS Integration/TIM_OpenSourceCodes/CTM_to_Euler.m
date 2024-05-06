function eul = CTM_to_Euler(C)
%CTM_to_Euler - Converts a coordinate transformation matrix to the
%corresponding set of Euler angles%
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   C       coordinate transformation matrix describing transformation from
%           beta to alpha
%
% Outputs:
%   eul     Euler angles describing rotation from beta to alpha in the 
%           order roll, pitch, yaw(rad)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

% Calculate Euler angles using (2.23)
eul(1,1) = atan2(C(2,3),C(3,3));  % roll
eul(2,1) = - asin(C(1,3));        % pitch
eul(3,1) = atan2(C(1,2),C(1,1));  % yaw

% Ends