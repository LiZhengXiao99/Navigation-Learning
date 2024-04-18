% ------------------------------------------------------------------- 
% Model perpesents a pulsed radar tracking system. 
%   Radar pulses are sent out and return signals are processed by
%   the KF in order to determine the position of maneuvering airborne objects
% ------------------------------------------------------------------- 
% References:
%   Example 5.8 
%   M. Grewal, A. Andrews, Kalman filtering: theory and practice using
%        MATLAB, 4th Edition, John Wiley & Sons, New Jersey, 2015.
%   R. A. Singer, "Estimating optimal tracking filter performance for 
%       manned maneuvering targets," IEEE Transactions on Aerospace 
%       and Electronic Systems, Vol. AES - 6, pp. 473-483, 1970.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_tracking

  T = 2; % 10, 15 sampling time
ro = 0.5; 
% ---- process equation --------------------------------------
Fsys = [1,T,0,0,0,0;
        0,1,1,0,0,0;
        0,0,ro,0,0,0;
        0,0,0,1,T,0;
        0,0,0,0,1,1;
        0,0,0,0,0,ro;];

sigma1 = 103/3; sigma2 = 1.3e-8;
Gsys = [0,0;
        0,0;
        1,0;
        0,0;
        0,0;
        0,1;];
Qsys = [sigma1^2, 0; 
        0, sigma2^2;];

% ---- measurement equation -----------------------------------
sigma_theta = 1000;
sigma_r     = 0.017;
Hsys = [1,0,0,0,0,0;0,0,0,1,0,0];  
Rsys = [sigma_r^2, 0; 0, sigma_theta^2]; 

% ---- filter initials     -------------------------------------
P0   = [sigma_r^2,     (sigma_r^2)/T,              0,        0,               0, 0;
        (sigma_r^2)/T, 2*sigma_r^2/(T^2)+sigma1^2, 0,        0,               0, 0;
                    0,                          0, sigma1^2, 0,               0, 0;
                    0,                          0,        0, sigma_theta^2,   sigma_theta^2/T, 0;
                    0,                          0,        0, sigma_theta^2/T, 2*sigma_theta^2/(T^2)+sigma2^2, 0;
                    0,                          0,        0,               0,                              0,sigma2^2;];
x0 = [0;0;0;0;0;0;]; 
end
