% ------------------------------------------------------------------- 
% original Maximum Correntropy Criterion Kalman Filter (MCC-KF)
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: Two stages, a posteriori form
% Recursion: Riccati-type underlying recursion
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
% 1. Maximum correntropy Criterion Kalman filter (MCC-KF) is originally derived in
%    R. Izanloo, S. A. Fakoorian, H. S. Yazdi, D. Simon, Kalman filtering
%    based on the maximum correntropy criterion in the presence of non-
%    Gaussian noise, in: 2016 Annual Conference on Information Science and
%    Systems (CISS), 2016, pp. 500-505.
%    DOI:  https://doi.org/10.1109/CISS.2016.7460553 
%
% 2. Codes was also available at http://embeddedlab.csuohio.edu/Correntropy
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,G, etc
%     initials_filter - initials x0,P0
%     measurements    - measurement history
% Output:
%     PI          - performance index (Baram Proximity Measure here)
%     hatX        - estimates (history) 
%     hatDP       - diag of the filtered error covariance (history)
% ------------------------------------------------------------------- 
function [PI,hatX,hatDP] = Riccati_MCCKF(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
  
        [m,n]  = size(H);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         hatX  = zeros(n,N_total+1);      % prelocate for efficiency
         hatDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI

 hatX(:,1)  = X; hatDP(:,1) = diag(P);    % save initials at the first entry 
 for k = 1:N_total                
      [X,P]    = kf_predict(X,P,F,G,Q);                            
      lambda_k = feval(handle_kernel,matrices,X,P,measurements(:,k));
      if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The MCC-KF estimator implies a scalar adjusting parameter'); end;
      Rek_lambda  = R + lambda_k*H*P*H';  % only for PI calculation
      [X,P,ek] = mcckf_update(X,P,measurements(:,k),H,R,lambda_k);       

      PI = PI + 1/2*log(det(Rek_lambda))+1/2*ek'/Rek_lambda*ek; 
      hatX(:,k+1)  = X; hatDP(:,k+1) = diag(P);    % save  information
  end;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P] = kf_predict(X,P,F,G,Q)
    X = F*X;              % Predicted state estimate  
    P = F*P*F' + G*Q*G';  % Predicted error covariance 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P,residual] = mcckf_update(X,P,z,H,R,lambda_k)
  residual     = z - H*X;                     % residual
  invR = inv(R);                              % compute once

  Gain = (inv(P)+ lambda_k*H'*invR*H)\(lambda_k*H'*invR);
  A = (eye(size(P,1)) - Gain*H);
  P = A*P*A' + Gain*R*Gain';      % Joseph Stabilized form for P
  X = X + Gain * residual;        % Filtered state estimate
end
