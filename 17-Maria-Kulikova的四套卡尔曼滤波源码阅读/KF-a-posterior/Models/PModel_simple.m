% Simple test problem
%
% References:
%   Bierman G.J., Belzer M.R., Vandergraft J.S. and Porter D.W., 
%   "Maximum likelihood estimation using square root information filters",
%   IEEE Trans. Automat. Contr., vol. 35, no. 12, pp. 1293–1298, 1990.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0,parameters] = PModel_simple(delt)

% ---- model parameters  -------------------------------------
tau        = sym('tau','real'); % model parameters
parameters = {'tau'};

% ---- process equation --------------------------------------
Fsys = [1,  delt; 0, exp(-delt/tau)]; 
Gsys = [0; 1]; 
Qsys = [1];

% ---- measurement equation -----------------------------------
Hsys = [1, 0];  
Rsys = [1]; 

% ---- filter initials  -------------------------------------
P0 =  eye(2);
x0 =  zeros(2,1);

end
