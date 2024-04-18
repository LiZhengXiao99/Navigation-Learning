% ------------------------------------------------------------------- 
% Extended Square-root Covariance Kalman Filter 
%         Method: Cholesky-based implementation with upper triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form, extended form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   For a lower triangular factors, the method has been proposed as 
%   a part of combined algorithm on p.899 of the manuscript
%   1. Park P., Kailath T. (1995) New square-root algorithms for Kalman 
%      filtering, IEEE Trans. Automat. Contr. 40(5):895-899.
%      DOI: 10.1109/9.384225 
%   For an upper triangular factors, the method has been presented in
%   2. Kulikova M.V. (2009) Maximum likelihood estimation via the
%      extended covariance and combined square-root filters, 
%      Mathematicsand Computers in Simulation, 79(5): 1641-1657
%      DOI: http://dx.doi.org/10.1016/j.matcom.2008.08.004 
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
function [neg_LLF,hatX,hatDP] = Riccati_KF_eSRCF_QR(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % get initials for the filter 
          
        [m,n]  = size(H);                % read off dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), sqrtQ = diag(sqrt(diag(Q))); else  sqrtQ = chol(Q,'upper'); end; clear Q; 
        if isequal(diag(diag(R)),R), sqrtR = diag(sqrt(diag(R))); else  sqrtR = chol(R,'upper'); end; clear R; 
        if isequal(diag(diag(P)),P), sqrtP = diag(sqrt(diag(P))); else  sqrtP = chol(P,'upper'); end;   

    neg_LLF = 1/2*m*log(2*pi)*N_total;   % set initial value for the neg Log LF
 hatX(:,1)  = X; hatDP(:,1) = diag(P);   % save initials at the first entry
         PX = (sqrtP')\X;                % set te initials for the filter 
for k = 1:N_total                 
   [PX,sqrtP]                  = esrcf_predict(PX,sqrtP,F,G,sqrtQ); 
   [PX,sqrtP,norm_ek,sqrt_Rek] = esrcf_update(PX,sqrtP,measurements(:,k),H,sqrtR);
   
   neg_LLF = neg_LLF + 1/2*log(det(sqrt_Rek'*sqrt_Rek)) + 1/2*(norm_ek')*norm_ek;
   hatX(:,k+1) = sqrtP'*PX; hatDP(:,k+1) = diag(sqrtP'*sqrtP); % save estimates   
 end;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,sqrtP] = esrcf_predict(PX,sqrtP,F,G,sqrtQ)
     [n,q]      = size(G);
     PreArray   = [sqrtP*F', PX; sqrtQ*G', zeros(q,1);];

   % --- triangularize the first two (block) columns only ---          
   [Oth,PostArray] = qr(PreArray(:,1:n));  

   % --- transform the last column via the same orthogonal transformation
    PostArrayRest  = Oth'*PreArray(:,n+1:end); 

   % -- read-off the resulted quantities -------------------
    sqrtP        = PostArray(1:n,1:n);    % Predicted factor of P        
    PX           = PostArrayRest(1:n,:);  % Predicted PX element        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [PX,sqrtP,norm_residual,sqrtRe] = esrcf_update(PX,sqrtP,z,H,sqrtR)
    [m,n]     = size(H);
    PreArray  = [sqrtR,    zeros(m,n),  -sqrtR'\z; 
                 sqrtP*H', sqrtP,        PX;];

   % --- triangularize the first two (block) columns only ---          
   [Oth,PostArray] = qr(PreArray(:,1:m+n));  

   % --- transform the last column via the same orthogonal transformation
    PostArrayRest  = Oth'*PreArray(:,m+n+1:end); 

   % -- read-off the resulted quantities -------------------
           sqrtRe  =  PostArray(1:m,1:m);         % Filtered factor of R_{e,k}             
     norm_residual = -PostArrayRest(1:m,:);       % normalized innovations 
                PX =  PostArrayRest(1+m:m+n,:);   % Filtered PX element
             sqrtP =  PostArray(m+1:m+n,m+1:m+n); % Filtered factor of P
end
