% ------------------------------------------------------------------- 
% SVD-based Covariance Kalman Filter 
%         Method: SVD-based implementation
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form
%        Authors: L. Wang, G. Libert, and P. Manneback (1992)
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   L. Wang, G. Libert, and P. Manneback (1992) "Kalman Filter Algorithm
%   based on Singular Value Decomposition", Proceedings of the 31st
%   Conference on Decision and Control. Tuczon, AZ, USA: IEEE, 1992,
%   pp. 1224-1229. See section "Algorithmic detailes". eq. (17), (22), (23) 
%   DOI: 10.1109/CDC.1992.371522 
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
function [neg_LLF,hatX,hatDP] = Riccati_KF_SVDSR(matrices,initials_filter,measurements)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
         [X,P] = deal(initials_filter{:});  % initials for the filter 
          
        [m,n]  = size(H);                % dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

        if isequal(diag(diag(Q)),Q), sqrtQ = diag(sqrt(diag(Q))); else  sqrtQ = chol(Q,'upper'); end; clear Q; 
        if isequal(diag(diag(R)),R), sqrtR = diag(sqrt(diag(R))); else  sqrtR = chol(R,'upper'); end; clear R; 
        [~,DP,QP] = svd(P); DPsqrt = DP.^(1/2); clear DP; 

   neg_LLF = 1/2*m*log(2*pi)*N_total;                    % initial value for the neg Log LF
   hatX(:,1)  = X; hatDP(:,1) = diag(QP*DPsqrt^(2)*QP'); % save initials at the first entry
 for k = 1:N_total      
   [X,DPsqrt,QP]        = svd_sr_predict(X,DPsqrt,QP,F,G,sqrtQ); 
   [X,DPsqrt,QP,ek,Rek] = svd_sr_update(X,DPsqrt,QP,measurements(:,k),H,sqrtR);

   neg_LLF = neg_LLF+1/2*log((det(Rek)))+1/2*ek'/Rek*ek;
   hatX(:,k+1)= X; hatDP(:,k+1) = diag(QP*(DPsqrt^2)*QP'); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP] = svd_sr_predict(X,DPsqrt,QP,F,G,Q_sqrt)
     [n,~]    = size(G);
     PreArray = [DPsqrt*QP'*F'; Q_sqrt*G';];
          
     [~,S,QP] = svd(PreArray); % Predicted SVD factors of P        
       DPsqrt = S(1:n,1:end);  % Predicted SVD factors of P     
            X = F*X;           % Predicted state estimate   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP,ek,Rek] = svd_sr_update(X,DPsqrt,QP,z,H,R_sqrt)
      [~,n]     = size(H);
            Rek = R_sqrt'*R_sqrt + H*(QP*(DPsqrt^2)*QP')*H';  % residual covariance for LLF

     PreArray   = [R_sqrt'\H*QP; inv(DPsqrt);];
     [~,Sm,Vm]  = svd(PreArray);  
         DPsqrt = inv(Sm(1:n,1:end));                   % Filtered SVD factors of P
             QP = QP*Vm;                                % Filtered SVD factors of P                               
 
     Gain = (QP*DPsqrt*DPsqrt*QP')*H'/R_sqrt'/R_sqrt;   % Kalman Gain
       ek = z-H*X;                                      % residual for LLF
        X = X + Gain*ek;                                % Filtered state estimate        
end
     