% ------------------------------------------------------------------- 
% Model is developed by  K. Suotsalo and S. Sarkka (2017). 
%
% References:
%    K. Suotsalo and S. Sarkka (2017). 
%    "A linear stochastic state space model for electrocardiograms". 
%    Proceedings of 27th IEEE International Workshop on Machine 
%    Learning for Signal Processing (MLSP), 2017. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_electrocardiogram

delta_t = 0.1;  % measured at time intervals of t = 1/10 seconds

% ---- process equation --------------------------------------
Fsys = [1 delta_t 1/2*delta_t^2; 0 1 delta_t; 0 0 1;];  
Qsys = [1/20*(delta_t)^5, 1/8*(delta_t)^4, 1/6*(delta_t)^3;
        1/8*(delta_t)^4, 1/3*(delta_t)^3, 1/2*(delta_t)^2;
        1/6*(delta_t)^3, 1/2*(delta_t)^2, delta_t];
Gsys = eye(3);

% ---- measurement equation -----------------------------------
Hsys = [1 0 0]; 
Rsys = 0.01;

% ---- filter initials     -------------------------------------
x0 = [1;0.1;0];           % initial of state vector X
P0 = diag([0.1,0.1,0.1]); % initial of cov matrix P

end

