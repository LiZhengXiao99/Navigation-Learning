% ------------------------------------------------------------------- 
% Extended Square-root Covariance Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%      Type: Covariance filtering
%    Method: Cholesky-based implementation with upper triangular factors
%      From: Two stages, a posteriori form, extended array form
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
%
% References:
% 1. This is the implementation of Algorithm 3 from the following paper:
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
function [PI,hatX,hatDP] = Riccati_IMCCKF_eSRCF_QR(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency
            PI = 0;                      % set initial value for the PI

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'upper'); end; clear Q; 
        if isequal(diag(diag(R)),R), sqrtR = diag(sqrt(diag(R))); else  sqrtR = chol(R,'upper'); end; 
        if isequal(diag(diag(P)),P), sqrtP = diag(sqrt(diag(P))); else  sqrtP = chol(P,'upper'); end;   
        GQsqrt = G*Q_sqrt';          % compute once the new G*Q^{1/2}
            PX = (sqrtP')\X;         % initials for the filter 
        
hatX(:,1) = X; hatDP(:,1) = diag(P); % save initials at the first entry
for k = 1:N_total                 
   [PX,sqrtP]   = esrcf_predict(PX,sqrtP,F,GQsqrt); 
      lambda_k  = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;
   [PX,sqrtP,norm_ek,sqrt_Rek] = esrcf_update(PX,sqrtP,measurements(:,k),H,sqrtR,lambda_k);
   
   PI = PI + 1/2 * log(det(sqrt_Rek'*sqrt_Rek))+1/2*(norm_ek')*norm_ek;
   hatX(:,k+1) = sqrtP'*PX; hatDP(:,k+1) = diag(sqrtP'*sqrtP); % save estimates   
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,sqrtP] = esrcf_predict(PX,sqrtP,F,G)
     [n,q]      = size(G);
     PreArray   = [sqrtP*F', PX; G', zeros(q,1);];
   % --- triangularize the first two (block) columns only ---          
   [Oth,PostArray] = qr(PreArray(:,1:n));  
    PostArrayRest  = Oth'*PreArray(:,n+1:end); 
   % -- read-off the resulted quantities -------------------
    sqrtP        = PostArray(1:n,1:n);    % Predicted factor of P        
    PX           = PostArrayRest(1:n,:);  % Predicted PX element        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,sqrtP,norm_residual,sqrtRe] = esrcf_update(PX,sqrtP,z,H,sqrtR,lambda_k)
    [m,n]     = size(H);
    PreArray  = [sqrtR,    zeros(m,n),  -sqrtR'\z*sqrt(lambda_k); 
                 sqrt(lambda_k)*sqrtP*H', sqrtP,        PX;];
   % --- triangularize the first two (block) columns only ---          
   [Oth,PostArray] = qr(PreArray(:,1:m+n));  
    PostArrayRest  = Oth'*PreArray(:,m+n+1:end); 
   % -- read-off the resulted quantities -------------------
           sqrtRe  =  PostArray(1:m,1:m);                     % Filtered factor of R_{e,k}             
     norm_residual = -PostArrayRest(1:m,:)/sqrt(lambda_k);    % normalized innovations 
                PX =  PostArrayRest(1+m:m+n,:);               % Filtered PX element
             sqrtP =  PostArray(m+1:m+n,m+1:m+n);             % Filtered factor of P
end
