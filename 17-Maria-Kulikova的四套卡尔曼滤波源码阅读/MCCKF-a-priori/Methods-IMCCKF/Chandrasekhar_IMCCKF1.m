% ------------------------------------------------------------------- 
% Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: One stage, a priori form
% Recursion: Chandrasekhar-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
% 1. This is Algorithm 1 [based on recursion (23)] in the following paper:
%    Kulikova M.V. (2020) 
%    Chandrasekhar-based maximum correntropy Kalman filtering with 
%    the adaptive kernel size selection. IEEE Transactions on Automatic
%    Control, 65(2): 741-748. 
%    DOI: https://doi.org/10.1109/TAC.2019.2919341
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,H,Q etc
%     initials_filter - initials x0,P0
%     measurements    - measurements (where y(t_k) is the k-th column)
% Output:
%     PI          - performance index (Baram Proximity Measure here)
%     predX       - a priori estimates (history) 
%     predDP      - diag of the predicted error covariance (history)
% ------------------------------------------------------------------- 
function [PI,predX,predDP] = Chandrasekhar_IMCCKF1(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
   
        [m,n]  = size(H);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         predX = zeros(n,N_total+1);      % prelocate for efficiency
        predDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI

predX(:,1) = X; predDP(:,1) = diag(P);    % save initials at the first entry
lambda     = feval(handle_kernel,matrices,X,P,measurements(:,1));
if (size(lambda,1)>1)||(size(lambda,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
% --- Filter initials --------------------------------------------------------
Rek    = R + lambda*H*P*H';                       % initial residual covariance matrix 
K_pk   = F*P*H'/Rek;                              % initial gain
Delta0 = F*P*F'+G*Q*G'-lambda*K_pk*Rek*K_pk'-P;   % initial difference
% ------ Reduced Row Echelon Form ---------------------------------------------
[V1,D1] = eig(Delta0);        % may check that norm(V1*D1*V1' - Delta0) approx 0
[~,bascolumn] = rref(Delta0); % norm(V1(:,bascolumn)*D1(bascolumn,bascolumn)*V1(:,bascolumn)' - Delta0) approx 0
alpha = length(bascolumn); L = V1(:,bascolumn); M = D1(bascolumn,bascolumn);

% --- Filtering------ --------------------------------------------------------
for k = 1:N_total  
     lambda_new = feval(handle_kernel,matrices,X,P,measurements(:,k));
     if (lambda_new~=lambda), error('The Chandrasekhar recursion is derived for constant adjusting parameter, only'); end;

      ek = measurements(:,k) - H*X;                             % residual
       X = F*X + lambda*K_pk*ek;                                % a priori estimate X
      PI = PI + 1/2*log(det(Rek))+1/2*(ek'/Rek)*ek;             % performance index
     predX(:,k+1) = X; P = P + L*M*L'; predDP(:,k+1) = diag(P); % save estimates

     %--- recursion for new L and M ----------------
     Rek_next = Rek + lambda*H*L*M*L'*H'; 
        K_pk  = (K_pk*Rek + F*L*M*L'*H')/Rek_next;     % inverse new Rek, i.e. the matrix of size m
       L_next = (F-lambda*K_pk*H )*L;
            M = M + lambda*M*L'*H'/Rek*H*L*M;          % inverse old Rek, i.e. the matrix of size m
            L = L_next; Rek = Rek_next;                % new iteration starts
 end;
end



 
