% ------------------------------------------------------------------- 
% Square-root Covariance Maximum Correntropy Criterion Kalman Filter (MCC-KF)
%      Type: Covariance filtering
%    Method: Cholesky-based implementation with upper triangular factors
%      From: Two stages, a posteriori form
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
% 1. This is the implementation of Algorithm 1a from the following paper: 
%    Kulikova M.V. (2019) Factored-form Kalman-like Implementations under 
%    Maximum Correntropy Criterion. Signal Processing. 
%    DOI: https://doi.org/10.1016/j.sigpro.2019.03.003
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,G etc
%     initials_filter - initials x0,P0
%     measurements    - measurement history
% Output:
%     PI          - performance index (Baram Proximity Measure here)
%     hatX        - estimates (history) 
%     hatDP       - diag of the filtered error covariance (history)
% ------------------------------------------------------------------- 
function [PI,hatX,hatDP] = Riccati_MCCKF_SRCF_QR(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency
            PI = 0;                      % set initial value for the PI

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'upper'); end; clear Q; 
        if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'upper'); end; 
        if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'upper'); end;   
        GQsqrt = G*Q_sqrt';          % compute once the new G*Q^{1/2}

 hatX(:,1) = X; hatDP(:,1) = diag(P); % save initials at the first entry
 for k = 1:N_total                  
   [X,P_sqrt] = srcf_predict(X,P_sqrt,F,GQsqrt);  
      lambda_k = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The MCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;
   Rek_lambda  = R + lambda_k*H*(P_sqrt'*P_sqrt)*H';  % only for PI calculation
   [X,P_sqrt,ek] = sr_mcckf_update(X,P_sqrt,measurements(:,k),H,R_sqrt,lambda_k);

    PI = PI + 1/2*log(det(Rek_lambda))+1/2*ek'/Rek_lambda*ek; 
    hatX(:,k+1)  = X; hatDP(:,k+1) = diag(P_sqrt'*P_sqrt);    % save  information
 end;
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Additional functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP] = srcf_predict(X,sqrtP,F,GQsqrt)
     [n,~]      = size(F);
     PreArray   = [sqrtP*F'; GQsqrt';];
          
    [~,PostArray]  = qr(PreArray);
       sqrtP       = PostArray(1:n,1:n);
       X           = F*X;
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP,residual] = sr_mcckf_update(X,sqrtP,z,H,sqrtR,lambda_k)
     [m,n]  = size(H);
   residual = z - H*X;   
  PreArray  = [inv(sqrtP'); sqrt(lambda_k)*inv(sqrtR)'*H;];
   [~,PostArray]  = qr(PreArray);
       sqrtPinv   = (PostArray(1:n,1:n))';

   Gain = lambda_k*inv(sqrtPinv*sqrtPinv')*H'*inv(sqrtR'*sqrtR);
      X = X + Gain * residual;

% the MCC-KF method implies Joseph stabilized formula for updating the error covariance matrix at the end
 
        PreArray  = [sqrtP*(eye(n) - Gain*H)'; sqrtR*Gain';];
   [~,PostArray]  = qr(PreArray);
          sqrtP   = PostArray(1:n,1:n);
end
