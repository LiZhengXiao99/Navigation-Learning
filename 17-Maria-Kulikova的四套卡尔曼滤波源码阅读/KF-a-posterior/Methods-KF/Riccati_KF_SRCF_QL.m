% ------------------------------------------------------------------- 
% Square-root Covariance Kalman Filter 
%         Method: Cholesky-based implementation with lower triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   The method has appeared many times in the literature, e.g., see
%   1. Kaminski P.G., Bryson A.E., Schmidt S.F. (1971) "Discrete square-root
%      filtering: a survey of current techniques", IEEE Trans. Automat.
%      Control. AC-16(6): 727–735.   DOI: 10.1109/TAC.1971.1099816 
%   2. Park P., Kailath T. (1995) New square-root algorithms for Kalman 
%      filtering, IEEE Trans. Automat. Contr. 40(5):895-899.
%      DOI: 10.1109/9.384225 
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,H,Q etc
%     initials_filter - initials x0,P0
%     measurements    - measurements (where y(t_k) is the k-th column)
% Output:
%     neg_LLF     - negative log LF
%     hatX        - filtered estimate (history) 
%     hatDP       - diag of the filtered error covariance (history)
% ------------------------------------------------------------------- 
function [neg_LLF,hatX,hatDP] = Riccati_KF_SRCF_QL(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'lower'); end; clear Q; 
        if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'lower'); end; clear R; 
        if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'lower'); end;   
        
  neg_LLF = 1/2*m*log(2*pi)*N_total;   % initial value for the neg Log LF
  hatX(:,1) = X; hatDP(:,1) = diag(P); % save initials at the first entry
for k = 1:N_total                  
   [X,P_sqrt]                  = srcf_predict(X,P_sqrt,F,G,Q_sqrt);  
   [X,P_sqrt,norm_ek,sqrt_Rek] = srcf_update(X,P_sqrt,measurements(:,k),H,R_sqrt);
   
   neg_LLF = neg_LLF + 1/2*log(det(sqrt_Rek*sqrt_Rek')) + 1/2*(norm_ek')*norm_ek;
   hatX(:,k+1) = X; hatDP(:,k+1) = diag(P_sqrt*P_sqrt'); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP] = srcf_predict(X,sqrtP,F,G,Q_sqrt)
     [n,~]      = size(sqrtP);
     PreArray   = [F*sqrtP, G*Q_sqrt;];
 
   % --- triangularize the pre-array ---          
    [~,PostArray]  = qr(PreArray');
   % --- lower triangular post-arrays:
         PostArray = PostArray';
   % -- read-off the resulted quantities -------------------
       sqrtP       = PostArray(1:n,1:n); % Predicted factor of P        
 
       X           = F*X;                % Predicted state estimate   
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP,norm_residual,sqrtRe] = srcf_update(X,sqrtP,z,H,R_sqrt)
    [m,n]     = size(H);
    PreArray  = [R_sqrt,    H*sqrtP; 
                 zeros(n,m), sqrtP;];

   % --- triangularize the pre-array ---          
    [~,PostArray]  = qr(PreArray');
   % --- lower triangular post-arrays:
         PostArray = PostArray';

   % -- read-off the resulted quantities -------------------
           sqrtRe  = PostArray(1:m,1:m);          % Filtered factor of R_{e,k}           
     norm_residual = sqrtRe\(z-H*X);              % normalized innovations
             sqrtP = PostArray(m+1:m+n,m+1:m+n);  % Filtered factor of P           
   norm_KalmanGain = PostArray(m+1:m+n,1:m);      % normalized Kalman gain

                 X = X + norm_KalmanGain*norm_residual; % Filtered estimate
end
