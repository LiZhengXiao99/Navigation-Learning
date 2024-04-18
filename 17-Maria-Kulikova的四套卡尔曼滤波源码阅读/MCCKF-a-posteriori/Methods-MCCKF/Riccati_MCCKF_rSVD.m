% ------------------------------------------------------------------- 
% robust SVD-based Maximum Correntropy Criterion Kalman Filter (MCC-KF)
%      Type: Covariance filtering
%    Method: SVD-based implementation
%      From: Two stages, a posteriori form, column representation of arrays
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
% 1. This is the implementation of Algorithm 1d from the following paper: 
%    Kulikova M.V. (2019) Factored-form Kalman-like Implementations under 
%    Maximum Correntropy Criterion. Signal Processing. 160: 328-338. 
%    DOI: https://doi.org/10.1016/j.sigpro.2019.03.003
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
function [PI,hatX,hatDP] = Riccati_MCCKF_rSVD(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Qsys,H,R] = deal(matrices{:});       % get system matrices
          [X,P] = deal(initials_filter{:});  % initials for the filter 
   
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency
            PI = 0;                      % set initial value for the PI

     [~,DQ,QQ] = svd(Qsys);      % SVD for process noise covariance
        GQsqrt = G*QQ*DQ.^(1/2); % compute once the new G*Q^{1/2}
     [~,DR,RQ] = svd(R);         % SVD for measurement noise covariance
        DRsqrt = DR.^(1/2); clear DR; R_sqrt = DRsqrt*RQ'; % compute once

[~,DP,QP] = svd(P); DPsqrt = DP.^(1/2); clear DP;     % initials for the filter 
hatX(:,1)  = X; hatDP(:,1) = diag(QP*DPsqrt^(2)*QP'); % save initials at the first entry
for k = 1:N_total                       
   [X,DPsqrt,QP] = svd_predict(X,DPsqrt,QP,F,GQsqrt); 
        lambda_k = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The MCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;
   [X,DPsqrt,QP,ek,DRek,QRek] = svd_update(X,DPsqrt,QP,measurements(:,k),H,R_sqrt,lambda_k);
   
   PI = PI + 1/2*log(det(DRek)) + 1/2*ek'*QRek/DRek*QRek'*ek; 
   hatX(:,k+1)= X; hatDP(:,k+1) = diag(QP*DPsqrt^(2)*QP'); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP] = svd_predict(X,DPsqrt,QP,F,G)
     [n,~]      = size(G);
     PreArray   = [DPsqrt*QP'*F'; G';];
          
     [~,S,QP] = svd(PreArray); % Predicted SVD factors of P     
       DPsqrt = S(1:n,1:end);  % Predicted SVD factors of P 
            X = F*X;           % Predicted state estimate  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP,residual,DRek,QRek] = svd_update(X,DPsqrt,QP,z,H,sqrtR,Lk)
      [m,n]     = size(H);
    PreArray    = [sqrtR; sqrt(Lk)*DPsqrt*QP'*H';];
     [~,S,QRek] = svd(PreArray);                     % SVD factors of R_{e,k}  
           DRek = S(1:m,1:end)^2;                    % SVD factors of R_{e,k}
    
     residual  =  z-H*X;                                  % residual
          Gain = Lk*QP*(DPsqrt^2)*QP'*H'*QRek/DRek*QRek'; % Kalman gain
             X = X + Gain*residual;                       % Filtered estimate 

    PreArray  = [DPsqrt*QP'*(eye(n) - Gain*H)'; sqrtR*Gain';];
      [~,S,QP] = svd(PreArray);       
        DPsqrt = S(1:n,1:end);  % new DP_sqrt
end
