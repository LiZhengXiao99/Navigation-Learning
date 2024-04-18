% A randomly drifting stochastic resonator
%
% References:
%   S. Sarkka, A. Nummenmaa (2009). 
%   "Recursive Noise Adaptive Kalman Filtering by Variational Bayesian Approximations", 
%   IEEE Transactions on  Automatic Control, 54(3): 596--600.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_resonator

delta_t=1/10; % which is measured at time intervals of t = 1/10 seconds
omega= 0.05;  % angular velocity

Fsys=[1  0  0; 
      0  cos(omega*delta_t)  sin(omega*delta_t)/delta_t; 
      0  -omega*sin(omega*delta_t)  cos(omega*delta_t);];

Qsys = 0.01*eye(3);
Gsys = eye(3);

Hsys=[1  1 0];
Rsys = 0.2;

x0 = [0;0;0];               % initial of state vector X
P0 = diag([0.1,0.1,0.1]); % initial of cov matrix P
end



