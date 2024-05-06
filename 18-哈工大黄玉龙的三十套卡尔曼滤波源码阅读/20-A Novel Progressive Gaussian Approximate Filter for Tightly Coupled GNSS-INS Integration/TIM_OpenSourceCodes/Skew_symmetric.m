function A = Skew_symmetric(a)
%Skew_symmetric - Calculates skew-symmetric matrix
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   a       3-element vector
% Outputs:
%   A       3x3matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

A = [    0, -a(3),  a(2);...
      a(3),     0, -a(1);...
     -a(2),  a(1),     0];

% Ends