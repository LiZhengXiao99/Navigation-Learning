% ------------------------------------------------------------------- 
% classical Kalman Filter 
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: One stage, a priori form
% Recursion: Chandrasekhar-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%  The Algorithm is based on recursion (66) in the following paper (see eqs. (13)-(17)): 
%  1. Morf  M., Sidhu G. and  Kailath T. (1974) 
%     Some new algorithms for recursive estimation in constant, linear, discrete-time systems, 
%     IEEE Trans. Automat. Contr., vol. 19, no. 4, pp. 315-323, Aug. 1974.
%     DOI:   10.1109/TAC.1974.1100576  
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,H,Q etc
%     initials_filter - initials x0,P0
%     measurements    - measurements (where y(t_k) is the k-th column)
% Output:
%     neg_LLF     - negative log LF
%     predX       - a priori estimates (history) 
%     predDP      - diag of the predicted error covariance (history)
% ------------------------------------------------------------------- 
function [neg_LLF,predX,predDP] = Chandrasekhar_KF1(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
   
        [m,n]  = size(H);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         predX = zeros(n,N_total+1);      % prelocate for efficiency
        predDP = zeros(n,N_total+1);      % prelocate for efficiency
       neg_LLF = 1/2*m*log(2*pi)*N_total; % set initial value for the neg Log LF

predX(:,1) = X; predDP(:,1) = diag(P);    % save initials at the first entry
% --- Filter initials --------------------------------------------------------
Rek    = R + H*P*H';                       % initial residual covariance matrix 
K_pk   = F*P*H'/Rek;                       % initial gain
Delta0 = F*P*F'+G*Q*G'-K_pk*Rek*K_pk'-P;   % initial difference
% ------ Reduced Row Echelon Form ---------------------------------------------
[V1,D1] = eig(Delta0);        % may check that norm(V1*D1*V1' - Delta0) approx 0
[~,bascolumn] = rref(Delta0); % norm(V1(:,bascolumn)*D1(bascolumn,bascolumn)*V1(:,bascolumn)' - Delta0) approx 0
alpha = length(bascolumn); L = V1(:,bascolumn); M = D1(bascolumn,bascolumn);

% --- Filtering------ --------------------------------------------------------
for k = 1:N_total  
      ek = measurements(:,k) - H*X;                             % residual
       X = F*X + K_pk*ek;                                       % a priori estimate X
     neg_LLF = neg_LLF+1/2*log(det(Rek))+1/2*(ek'/Rek)*ek;      % negtive LLF
     predX(:,k+1) = X; P = P + L*M*L'; predDP(:,k+1) = diag(P); % save estimates

     %--- recursion for new L and M ----------------
     Rek_next = Rek + H*L*M*L'*H'; 
        K_pk  = (K_pk*Rek + F*L*M*L'*H')/Rek_next;     % inverse new Rek, i.e. the matrix of size m
       L_next = (F - K_pk*H )*L;
            M = M + M*L'*H'/Rek*H*L*M;                 % inverse old Rek, i.e. the matrix of size m
            L = L_next; Rek = Rek_next;                % new iteration starts
 end;
end



 
