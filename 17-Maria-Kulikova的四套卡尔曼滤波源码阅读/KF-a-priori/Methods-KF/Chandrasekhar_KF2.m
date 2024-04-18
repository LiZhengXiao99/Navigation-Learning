% ------------------------------------------------------------------- 
% classical Kalman Filter 
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: One stage, a priori form
% Recursion: Chandrasekhar-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
%  The Algorithm is based on recursion (67) in the following paper 
%     (see eqs. (13'), (14'), (15)-(17)): 
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
function [neg_LLF,predX,predDP] = Chandrasekhar_KF2(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
   
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
      ek = measurements(:,k) - H*X;                              % residual
       X = F*X + K_pk*ek;                                        % a priori estimate X
     neg_LLF = neg_LLF+1/2*log(det(Rek))+1/2*(ek'/Rek)*ek;       % negative LLF
     predX(:,k+1) = X; P = P + L*M*L'; predDP(:,k+1) = diag(P);  % save estimates

     %--- recursion for new L and M ----------------
    Rek_next = Rek + H*L*M*L'*H'; 
      L_next = (F - K_pk*H )*L;
inv_Rek_next = inv(Rek_next);                          % compute inverse matrix of size m, once    
       K_pk  = (K_pk*Rek + F*L*M*L'*H')*inv_Rek_next; 
           M = M - M*L'*H'*inv_Rek_next*H*L*M;                 
           L = L_next; Rek = Rek_next;                 % new iteration starts
end;
end



 
