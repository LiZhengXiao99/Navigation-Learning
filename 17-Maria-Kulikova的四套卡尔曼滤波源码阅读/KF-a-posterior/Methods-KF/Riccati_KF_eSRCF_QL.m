% ------------------------------------------------------------------- 
% Extended Square-root Covariance Kalman Filter 
%         Method: Cholesky-based implementation with lower triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form, extended form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   This is a part of combined algorithm on p.899 of the manuscript
%   1. Park P., Kailath T. (1995) New square-root algorithms for Kalman 
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
function [neg_LLF,hatX,hatDP] = Riccati_KF_eSRCF_QL(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
          
        [m,n]  = size(H);                % read off dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), sqrtQ = diag(sqrt(diag(Q))); else  sqrtQ = chol(Q,'lower'); end; clear Q; 
        if isequal(diag(diag(R)),R), sqrtR = diag(sqrt(diag(R))); else  sqrtR = chol(R,'lower'); end; clear R; 
        if isequal(diag(diag(P)),P), sqrtP = diag(sqrt(diag(P))); else  sqrtP = chol(P,'lower'); end;   

    neg_LLF = 1/2*m*log(2*pi)*N_total;   % set initial value for the neg Log LF
 hatX(:,1)  = X; hatDP(:,1) = diag(P);   % save initials at the first entry
         PX = sqrtP\X;                   % set initials for the filter 
for k = 1:N_total                 
   [PX,sqrtP]                  = esrcf_predict(PX,sqrtP,F,G,sqrtQ); 
   [PX,sqrtP,norm_ek,sqrt_Rek] = esrcf_update(PX,sqrtP,measurements(:,k),H,sqrtR);
   
   neg_LLF = neg_LLF + 1/2*log(det(sqrt_Rek*sqrt_Rek')) + 1/2*(norm_ek')*norm_ek;
   hatX(:,k+1) = sqrtP*PX; hatDP(:,k+1) = diag(sqrtP*sqrtP'); % save estimates   
 end;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,sqrtP] = esrcf_predict(PX,sqrtP,F,G,sqrtQ)
     [n,q]      = size(G);
     PreArray   = [F*sqrtP,  G*sqrtQ;
                   PX',       zeros(1,q);];

   % --- triangularize the first two (block) columns only ---          
   [Oth,PostArray] = qr(PreArray(1:n,:)');  

   % --- transform the last row via the same orthogonal transformation
       PostLastRow = Oth'*PreArray(end,:)'; 

   % --- lower triangular post-arrays:
       PostArray   = PostArray'; PostLastRow = PostLastRow';

   % -- read-off the resulted quantities -------------------
    sqrtP        = PostArray(1:n,1:n);     % Predicted factor of P        
    PX           = PostLastRow(end,1:n)';  % Predicted PX element   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,P_sqrt,bar_ek,Rek_sqrt] = esrcf_update(PX,P_sqrt,z,H,R_sqrt)
    [m,n]     = size(H); 

   % --- triangularize the first two (block) rows only ---          
        PreArray  = [R_sqrt,        H*P_sqrt; 
                     zeros(n,m),    P_sqrt;
                    -(R_sqrt\z)',  PX';];
    [Oth,PostArray]  = qr(PreArray(1:end-1,:)');

   % --- transform the last row via the same orthogonal transformation
         PostLastRow = Oth'*PreArray(end,:)'; 

   % --- lower triangular posr-arrays:
         PostArray  = PostArray'; PostLastRow = PostLastRow';

   % -- read-off the resulted quantities -------------------
        Rek_sqrt  = PostArray(1:m,1:m);
         % bar_K_pk = PostArray(m+1:m+n,1:m);      
           P_sqrt = PostArray(m+1:m+n,1+m:m+n);  
           bar_ek = -PostLastRow(1,1:m)';      
               PX = PostLastRow(1,m+1:m+n)';
end
