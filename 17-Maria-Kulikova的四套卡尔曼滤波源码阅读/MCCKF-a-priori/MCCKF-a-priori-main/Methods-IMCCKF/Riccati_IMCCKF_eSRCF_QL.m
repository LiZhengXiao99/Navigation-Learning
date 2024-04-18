% ------------------------------------------------------------------- 
% Extended Square-root Covariance Improved Maximum Correntropy Criterion Kalman Filter 
%         Method: Cholesky-based implementation with lower triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: One stage (condensed), a priori form
%         Author: Maria Kulikova 
% ------------------------------------------------------------------- 
% References: This is the implementation of Algorithm 2 from the following paper:
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
function [PI,predX,predDP] = Riccati_IMCCKF_eSRCF_QL(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
        [q,~]  = size(Q);                % dimensions
       N_total = size(measurements,2);   % number of measurements
         predX  = zeros(n,N_total+1);    % prelocate for efficiency
         predDP = zeros(n,N_total+1);    % prelocate for efficiency
             PI = 0;                       % set initial value for the PI

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'lower'); end; clear Q; 
        if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'lower'); end; clear R; 
        if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'lower'); end;   

PX = P_sqrt\X;                           % initials for the filter                        
predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
for k = 1:N_total           
   lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
   if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
   if lambda_k<0, error('The square-root MCC-KF implementations imply a non-negative adjusting parameter'); end;

   % --- triangularize the first two (block) rows only ---          
        PreArray  = [R_sqrt,                     sqrt(lambda_k)*H*P_sqrt, zeros(m,q); 
                     zeros(n,m),                                F*P_sqrt, G*Q_sqrt;
                  -(R_sqrt\measurements(:,k))'*sqrt(lambda_k),  PX',      zeros(1,q);];
    [Oth,PostArray]  = qr(PreArray(1:end-1,:)');
         PostLastRow = Oth'*PreArray(end,:)'; 
         PostArray   = PostArray'; PostLastRow = PostLastRow';

   % -- read-off the resulted quantities -------------------
        Rek_sqrt  = PostArray(1:m,1:m);
         %bar_K_pk = PostArray(m+1:m+n,1:m);      
           P_sqrt = PostArray(m+1:m+n,1+m:m+n);  
           bar_ek = -PostLastRow(1,1:m)'/sqrt(lambda_k);            
               PX = PostLastRow(1,m+1:m+n)';
                X = P_sqrt*PX;    
        
    PI = PI + log(det(Rek_sqrt))+1/2*bar_ek'*bar_ek; 
    predX(:,k+1) = X; predDP(:,k+1) = diag(P_sqrt*P_sqrt'); 
 end;
end

