% ------------------------------------------------------------------- 
% Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%         Method: Conventional implementation
%           Type: Covariance filtering
%      Recursion: Riccati underlying recursion
%           Form: One stage (condensed), a priori form
%        Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%    A priori form of the IMCC-KF. The derivation can be found in 
% 1. Kulikova M.V. (2019) 
%    One-Step Condensed Forms for Square-Root Maximum Correntropy Criterion 
%    Kalman Filtering, Proceedings of the 23rd International Conference on
%    System Theory, Control and Computing (ICSTCC),  Sinaia, Romania, pp. 13-18. 
%    Oct. 2019,  DOI: http://doi.org/10.1109/ICSTCC.2019.8885950
%
% 2. Kulikova M.V. (2020) 
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
function [PI,predX,predDP] = Riccati_IMCCKF_standard(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
   
        [m,n]  = size(H);                 % get dimensions
       N_total = size(measurements,2);    % number of measurements
         predX = zeros(n,N_total+1);      % prelocate for efficiency
        predDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI

 predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
for k = 1:N_total  
    lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
    if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The MCC-KF estimator implies a scalar adjusting parameter'); end;
            
    ek   = measurements(:,k) - H*X;   % residual
    Rek  = R + lambda_k*H*P*H';       % residual covariance matrix 
    Kpk  = F*P*H'/Rek;                % Gain matrix (predicted form)     
    
     X = F*X + lambda_k*Kpk*ek;                     % Predicted state estimate  
     P = F*P*F' + G*Q*G' - lambda_k*Kpk*Rek*Kpk';   % Predicted error covariance 
        
    PI = PI + 1/2*log(det(Rek))+1/2*ek'/Rek*ek; 
    predX(:,k+1) = X; predDP(:,k+1) = diag(P); 
 end;
end
