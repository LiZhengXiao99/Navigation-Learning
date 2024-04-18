% ------------------------------------------------------------------- 
% Extended Square-root Covariance Kalman Filter 
%         Method: Cholesky-based implementation with lower triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: One stage (condensed), a priori form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   1. This is the implementation of Algorithm III.2 from
%      Park P., Kailath T. (1995) New square-root algorithms for Kalman 
%      filtering. IEEE Transactions on Automatic Control, 40(5), 895-899.
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
function [neg_LLF,predX,predDP] = Riccati_KF_eSRCF_QL(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
        [q,~]  = size(Q);                % dimensions
       N_total = size(measurements,2);   % number of measurements
         predX  = zeros(n,N_total+1);    % prelocate for efficiency
         predDP = zeros(n,N_total+1);    % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'lower'); end; clear Q; 
        if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'lower'); end; clear R; 
        if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'lower'); end;   

neg_LLF = 1/2*m*log(2*pi)*N_total;       % initial values
PX = P_sqrt\X;                           % initials for the filter 
predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
for k = 1:N_total           
   % --- triangularize the first two (block) rows only ---          
        PreArray  = [R_sqrt,                        H*P_sqrt, zeros(m,q); 
                     zeros(n,m),                    F*P_sqrt, G*Q_sqrt;
                     -(R_sqrt\measurements(:,k))',  PX',      zeros(1,q);];
    [Oth,PostArray]  = qr(PreArray(1:end-1,:)');
         PostLastRow = Oth'*PreArray(end,:)'; 
         PostArray   = PostArray'; PostLastRow = PostLastRow';

   % -- read-off the resulted quantities -------------------
        Rek_sqrt  = PostArray(1:m,1:m);
         bar_K_pk = PostArray(m+1:m+n,1:m);      
           P_sqrt = PostArray(m+1:m+n,1+m:m+n);  
           bar_ek = -PostLastRow(1,1:m)';      
               PX = PostLastRow(1,m+1:m+n)';
                X = P_sqrt*PX;    
        
    neg_LLF = neg_LLF+log(det(Rek_sqrt))+1/2*bar_ek'*bar_ek; 
    predX(:,k+1) = X; predDP(:,k+1) = diag(P_sqrt*P_sqrt'); 
 end;
end

