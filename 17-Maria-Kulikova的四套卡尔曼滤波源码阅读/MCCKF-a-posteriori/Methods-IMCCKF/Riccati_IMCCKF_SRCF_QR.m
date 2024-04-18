% ------------------------------------------------------------------- 
% Square-root Covariance Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%      Type: Covariance filtering
%    Method: Cholesky-based implementation with upper triangular factors
%      From: Two stages, a posteriori form
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
%
% References:
% 1. This is the implementation of Algorithm 2 from the following paper:
%    Kulikova M.V. (2017) Square-root algorithms for maximum correntropy 
%    estimation of linear discrete-time systems in presence of non-Gaussian 
%    noise. Systems and Control Letters, 108: 8-15. 
%    DOI: https://doi.org/10.1016/j.sysconle.2017.07.016 
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
function [PI,hatX,hatDP] = Riccati_IMCCKF_SRCF_QR(matrices,initials_filter,measurements,handle_kernel)
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
       [X,P_sqrt]  = srcf_predict(X,P_sqrt,F,GQsqrt);  
      lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;
   [X,P_sqrt,norm_ek,sqrt_Rek] = srcf_update(X,P_sqrt,measurements(:,k),H,R_sqrt,lambda_k);
   
   PI = PI + 1/2*log(det(sqrt_Rek'*sqrt_Rek))+1/2*(norm_ek')*norm_ek; 
   hatX(:,k+1) = X; hatDP(:,k+1) = diag(P_sqrt'*P_sqrt); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP] = srcf_predict(X,sqrtP,F,G)
     [n,~]      = size(G);
     PreArray   = [sqrtP*F'; G';];
          
    [~,PostArray]  = qr(PreArray); 
       sqrtP       = PostArray(1:n,1:n); % Predicted factor of P        
       X           = F*X;                % Predicted state estimate   
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP,norm_residual,sqrtRe] = srcf_update(X,sqrtP,z,H,sqrtR,lambda_k)
    [m,n]     = size(H);
    PreArray  = [sqrtR,    zeros(m,n); 
                 sqrt(lambda_k)*sqrtP*H', sqrtP;];
        
    [~,PostArray]  = qr(PreArray);
           sqrtRe  = PostArray(1:m,1:m);           % Filtered factor of R_{e,k}           
     norm_residual = (sqrtRe')\(z-H*X);            % normalized innovations
             sqrtP =  PostArray(m+1:m+n,m+1:m+n);  % Filtered factor of P           
   norm_KalmanGain =  PostArray(1:m,m+1:m+n)';     % normalized gain
                 X =  X + sqrt(lambda_k)*norm_KalmanGain*norm_residual; % Filtered estimate
end
