% Benchmark navigation problem 
%
% References:
%    R. Izanloo, S. A. Fakoorian, H. S. Yazdi, D. Simon, Kalman filtering
%    based on the maximum correntropy criterion in the presence of non-
%    Gaussian noise, in: 2016 Annual Conference on Information Science and
%    Systems (CISS), 2016, pp. 500-505
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_navigation

T = 2;                                         % sampling time
Fsys = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1;];  % process Equation
Qsys = 0.1*eye(4);
Gsys = eye(4);

Hsys = [1 0 0 0; 0 1 0 0;];  % meaurment equation
Rsys = 0.1*eye(2);

x0 = [1;1;0;0];       % initial of state vector X
P0 = diag([4,4,3,3]); % initial of cov matrix P
end

