% ------------------------------------------------------------------- 
% Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: Two stages, a posteriori form
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova
% ------------------------------------------------------------------- 
% References:
% 1. IMCC-KF is designed in (Algorithm 1 in this paper)
%    Kulikova M.V. (2017) Square-root algorithms for maximum correntropy 
%    estimation of linear discrete-time systems in presence of non-Gaussian 
%    noise. Systems and Control Letters, 108: 8-15. 
%    DOI: https://doi.org/10.1016/j.sysconle.2017.07.016 
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,H,Q etc
%     initials_filter - initials x0,P0
%     measurements    - measurements (where y(t_k) is the k-th column)
% Output:
%     PI          - performance index (Baram Proximity Measure here)
%     hatX        - filtered estimate (history) 
%     hatDP       - diag of the filtered error covariance (history)
% ------------------------------------------------------------------- 
function [PI,hatX,hatDP] = Riccati_IMCCKF(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
   
        [m,n]  = size(H);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         hatX  = zeros(n,N_total+1);      % prelocate for efficiency
         hatDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI

 hatX(:,1)  = X; hatDP(:,1) = diag(P);    % save initials at the first entry 
 for k = 1:N_total                
      [X,P]        = kf_predict(X,P,F,G,Q);                            
      lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
      if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
      [X,P,ek,Rek] = imcckf_update(X,P,measurements(:,k),H,R,lambda_k);       
      
       PI = PI + 1/2*log(det(Rek)) + 1/2*ek'/Rek*ek; 
       hatX(:,k+1)  = X; hatDP(:,k+1) = diag(P);              
  end;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P] = kf_predict(X,P,F,G,Q)
    X = F*X;                  % Predicted state estimate  
    P = F*P*F' + G*Q*G'; % Predicted error covariance 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P,residual,cov_residual] = imcckf_update(X,P,z,H,R,lambda_k)
  residual     = z - H*X;                     % residual
  cov_residual = R + lambda_k*H*P*H';         % residual covariance matrix 
  Kalman_gain  = lambda_k*P*H'/cov_residual;  % Gain matrix

  X = X + Kalman_gain*residual;      % Filtered state estimate
  P = P-Kalman_gain*H*P;             % Filtered error covariance
end
