% ------------------------------------------------------------------- 
% Sequential Square-root Covariance Kalman Filter (component-wise measurement update)
%         Method: Cholesky-based implementation with upper triangular factors
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%    1. Kulikova M.V. (2009) 
%       On scalarized calculation of the likelihood function in 
%       array square-root filtering algorithms,
%       Automation and Remote Control, 70(5): 855-871.
%       DOI: http://dx.doi.org/10.1134/S0005117909050129
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
function [neg_LLF,hatX,hatDP] = Riccati_KF_SRCF_QR_seq(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

  fl=0; 
 if isequal(diag(diag(R)),R)==0                              % if R does not have a diag form,
    [H,R,measurements,fl] = DecorrelateR(H,R,measurements);  % one needs a decorrelation procedure
 end; 
 if isequal(diag(diag(Q)),Q), Q_sqrt = diag(sqrt(diag(Q))); else  Q_sqrt = chol(Q,'upper'); end; clear Q; 
 if isequal(diag(diag(R)),R), R_sqrt = diag(sqrt(diag(R))); else  R_sqrt = chol(R,'upper'); end; clear R; 
 if isequal(diag(diag(P)),P), P_sqrt = diag(sqrt(diag(P))); else  P_sqrt = chol(P,'upper'); end;   

neg_LLF = 1/2*m*log(2*pi)*N_total;   % initial value for the neg Log LF
hatX(:,1) = X; hatDP(:,1) = diag(P); % save initials at the first entry
for k = 1:N_total                  
   [X,P_sqrt]                  = srcf_predict(X,P_sqrt,F,G,Q_sqrt);  
   for i=1:m
     [X,P_sqrt,norm_ei,alph] = srcf_update(X,P_sqrt,measurements(i,k),H(i,:),R_sqrt(i,i)); 
     if fl==0, neg_LLF = neg_LLF + 1/2*log(alph^2) + 1/2*(norm_ei^2);   % if measurement model is changed, then the LLF should be also
                 else neg_LLF = NaN; end;                                  % transformed. For a fair comparison with other filters, we set the LLF to NaN.
   end;
   hatX(:,k+1) = X; hatDP(:,k+1) = diag(P_sqrt'*P_sqrt); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP] = srcf_predict(X,sqrtP,F,G,Q_sqrt)
     [n,~]      = size(G);
     PreArray   = [sqrtP*F'; Q_sqrt*G';];

   % --- triangularize the pre-array ---          
    [~,PostArray]  = qr(PreArray); 
   % -- read-off the resulted quantities -------------------
       sqrtP       = PostArray(1:n,1:n); % Predicted factor of P        

       X           = F*X;                % Predicted state estimate   
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,sqrtP,norm_residual,sqrtRe] = srcf_update(X,sqrtP,z,H,sqrtR)
    [m,n]     = size(H);
    PreArray  = [sqrtR,    zeros(m,n); 
                 sqrtP*H', sqrtP;];

   % --- triangularize the pre-array ---          
    [~,PostArray]  = qr(PreArray);
   % -- read-off the resulted quantities -------------------
           sqrtRe  = PostArray(1:m,1:m);           % Filtered factor of R_{e,k}           
     norm_residual = (sqrtRe')\(z-H*X);            % normalized innovations
             sqrtP =  PostArray(m+1:m+n,m+1:m+n);  % Filtered factor of P           
   norm_KalmanGain =  PostArray(1:m,m+1:m+n)';     % normalized Kalman gain

                 X =  X + norm_KalmanGain*norm_residual; % Filtered estimate
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement de-correlation:  based on Cholesky decomposition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Hsys,Rsys,meas,flag_tr] = DecorrelateR(Hsys,Rsys,meas)
flag_tr = 0; 
if all(all(Rsys==diag(diag(Rsys))))==0 % if R is not diagonal 
   L = chol(Rsys,'lower');            % (L*L')
   Hsys = L\Hsys; 
   Rsys = eye(size(Rsys,2)); 
   meas = L\meas;
   flag_tr  = 1;
end;
end
