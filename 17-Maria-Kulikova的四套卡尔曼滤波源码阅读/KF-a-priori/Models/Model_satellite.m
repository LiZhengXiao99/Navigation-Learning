% ------------------------------------------------------------------- 
% Model is studied by Rauch, H.E. and Tung, F, and Striebel, C.T. (1965). 
%   The dynamic system is a linearized version of the in-track motion 
%   of a satellite traveling in a circular orbit. 
% ------------------------------------------------------------------- 
% References:
%     Rauch, H.E. and Tung, F, and Striebel, C.T.
%     "Maximum Likelihood Estimates of Linear Dynamic Systems", 
%     AIAA JOURNAL, 8(3):1445-1450, 1965
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_satellite
               % Case 1 from the paper
q44 = 0.63e-2; % model parameters
p44 = 1e-2;    % model parameters

% ---- process equation --------------------------------------
Fsys = [1,1,0.5,0.5;0,1,1,1;0,0,1,0;0,0,0,0.606;]; 
Gsys = [0;0;0;1]; 
Qsys = [q44];

% ---- measurement equation -----------------------------------
Hsys = [1,0,0,0;];  
Rsys = 1; 

% ---- filter initials     -------------------------------------
P0 = eye(4); % [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,p44]; 
x0 = [0;0;0;0;]; 
end
