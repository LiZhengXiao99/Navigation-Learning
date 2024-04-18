% ------------------------------------------------------------------- 
% Extended Square-root Covariance Kalman Filter 
%         Method: Cholesky-based implementation with upper triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: One stage (condensed), a priori form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   1. This is the upper triangular variant of Algorithm III.2 from
%      Park P., Kailath T. (1995) New square-root algorithms for Kalman 
%      filtering. IEEE Transactions on Automatic Control, 40(5), 895-899.
%      DOI: 10.1109/9.384225 
%
%  This the upper triangular variant can be also found in
%   2. Kulikova M.V. (2009) Likelihood gradient evaluation 
%      using square-root covariance filters, 
%      IEEE Transactions on Automatic Control, 54(3): 646-651
%      DOI: http://dx.doi.org/10.1109/TAC.2008.2010989
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
function [neg_LLF,predX,predDP] = Riccati_KF_eSRCF_QR(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
        [q,~]  = size(Q);                % dimensions
       N_total = size(measurements,2);   % number of measurements
         predX  = zeros(n,N_total+1);    % prelocate for efficiency
         predDP = zeros(n,N_total+1);    % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'upper'); end; clear Q; 
        if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'upper'); end; clear R; 
        if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'upper'); end;   

neg_LLF = 1/2*m*log(2*pi)*N_total;       % initial values
PX = P_sqrt'\X;                          % initial values
predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
for k = 1:N_total             
   % --- triangularize the first two (block) columns only ---          
    PreArray  = [R_sqrt,      zeros(m,n),   -(R_sqrt')\measurements(:,k); 
                 P_sqrt*H',   P_sqrt*F',     PX;
                 zeros(q,m),  Q_sqrt*G',     zeros(q,1)];

   [Orth,PostArray1] = qr(PreArray(:,1:m+n));  % Q is any orthogonal rotation that upper-triangularizes the first
                                               % two block columns of the pre-array
   PostArray2 = Orth'*PreArray(:,end);         % transform the lat row as well 
   PostArray  = [PostArray1, PostArray2];      % the full post-array

   % -- read-off the resulted quantities -------------------
        Rek_sqrt  = PostArray(1:m,1:m);
           P_sqrt = PostArray(m+1:m+n,m+1:m+n);  
           bar_ek = -PostArray(1:m,end);      
               PX = PostArray(m+1:m+n,end);

    neg_LLF = neg_LLF + 1/2*log(det(Rek_sqrt'*Rek_sqrt)) + 1/2*(bar_ek')*bar_ek;
    predX(:,k+1) = P_sqrt'*PX; predDP(:,k+1) = diag(P_sqrt'*P_sqrt); 
 end;
end

