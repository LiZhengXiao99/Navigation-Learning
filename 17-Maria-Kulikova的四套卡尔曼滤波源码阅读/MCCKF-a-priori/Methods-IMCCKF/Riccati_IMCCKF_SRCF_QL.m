% Square-Root Covariance Improved Maximum Correntropy Criterion Kalman Filter
%      Type: Covariance filtering
%    Method: Cholesky-based implementation with lower triangular factors
%      From: One stage (condensed form), a priori form
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References: This is the implementation of Algorithm 1 from the following paper:
% 1. Kulikova M.V. (2019) 
%    One-Step Condensed Forms for Square-Root Maximum Correntropy Criterion 
%    Kalman Filtering, Proceedings of the 23rd International Conference on
%    System Theory, Control and Computing (ICSTCC),  Sinaia, Romania, pp. 13-18. 
%    Oct. 2019,  DOI: http://doi.org/10.1109/ICSTCC.2019.8885950
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
function [PI,predX,predDP] = Riccati_IMCCKF_SRCF_QL(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
   
        [m,n]  = size(H);                 % dimensions
        [n,q]  = size(G);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         predX = zeros(n,N_total+1);      % prelocate for efficiency
        predDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI

       if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'lower'); end; clear Q; 
       if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'lower'); end; clear R; 
       if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'lower'); end;   

 predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
 for k = 1:N_total
   lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;

    PreArray  = [R_sqrt,     sqrt(lambda_k)*H*P_sqrt, zeros(m,q); 
                 zeros(n,m),                F*P_sqrt, G*Q_sqrt;];
   % --- triangularize the pre-array ---          
    [~,PostArray]  = qr(PreArray');
   % --- lower triangular post-arrays:
         PostArray = PostArray';
   % -- read-off the resulted quantities -------------------
        Rek_sqrt  = PostArray(1:m,1:m);
         bar_K_pk = PostArray(m+1:m+n,1:m);      
           P_sqrt = PostArray(m+1:m+n,1+m:m+n);  
             ek   = measurements(:,k) - H*X;   % residual
                X = F*X + sqrt(lambda_k)*bar_K_pk/Rek_sqrt*ek;    
      
   PI = PI + 1/2*log(det(Rek_sqrt*Rek_sqrt')) + +1/2*ek'/Rek_sqrt'/Rek_sqrt*ek;
   predX(:,k+1) = X; predDP(:,k+1) = diag(P_sqrt*P_sqrt'); 
 end;
end
