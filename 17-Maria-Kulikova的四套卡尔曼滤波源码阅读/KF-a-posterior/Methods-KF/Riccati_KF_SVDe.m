% ------------------------------------------------------------------- 
% SVD-based Covariance Kalman Filter 
%         Method: SVD-based implementation, "economy size" SVD
%           Type: Covariance filtering
%      Recursion: Riccati recursion
%           Form: Two stages, a posteriori form
%        Authors: M.V. Kulikova, J.V. Tsyganova, G.Yu. Kulikov (2021)
% Implementation: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%   The SVD-based 'economy' size version with thin matrices (pre-arrays) has appeared in: 
%   1.  Kulikova M.V., Tsyganova J.V., Kulikov G.Yu. (2021) 
%       SVD-based state and parameter estimation approach for generalized 
%       Kalman filtering with application to GARCH-in-Mean estimation. 
%       Journal of Computational and Applied Mathematics, 387: 112487.
%       DOI: https://doi.org/10.1016/j.cam.2019.112487
%   The original SVD has been published in:
%   2.  Kulikova M.V., Tsyganova J.V. (2017) "Improved discrete-time Kalman 
%       filtering within singular value decomposition". 
%       IET Control Theory & Applications, 11(15): 2412-2418
%       DOI:  10.1049/iet-cta.2016.1282 
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

function [neg_LLF,hatX,hatDP] = Riccati_KF_SVDe(matrices,initials_filter,measurements)
   [F,G,Qsys,H,R] = deal(matrices{:});         % get system matrices
            [X,P] = deal(initials_filter{:});  % get initials for the filter 
   
        [m,n]  = size(H);                % get dimensions
       N_total = size(measurements,2);   % number of measurements
          hatX = zeros(n,N_total+1);     % prelocate for efficiency
         hatDP = zeros(n,N_total+1);     % prelocate for efficiency

     [~,DQ,QQ] = svd(Qsys);      % SVD for process noise covariance
        GQsqrt = G*QQ*DQ.^(1/2); % to compute once the new G*Q^{1/2}
     [~,DR,RQ] = svd(R);         % SVD for measurement noise covariance
        R_sqrt = RQ*DR.^(1/2); clear DR; % to compute once
     [~,DP,QP] = svd(P); DPsqrt = DP.^(1/2); clear DP;

   neg_LLF = 1/2*m*log(2*pi)*N_total;                 % initial value for the neg Log LF
hatX(:,1)  = X; hatDP(:,1) = diag(QP*(DPsqrt^2)*QP'); % save initials at the first entry
for k = 1:N_total                       
   [X,DPsqrt,QP]              = svd_predict(X,DPsqrt,QP,F,GQsqrt); 
   [X,DPsqrt,QP,ek,DRek,QRek] = svd_update(X,DPsqrt,QP,measurements(:,k),H,R_sqrt);
   
   neg_LLF = neg_LLF+1/2*log(det(DRek))+1/2*ek'*QRek/DRek*QRek'*ek; 
   hatX(:,k+1)= X; hatDP(:,k+1) = diag(QP*DPsqrt^(2)*QP'); % save estimates  
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP] = svd_predict(X,DPsqrt,QP,F,GQsqrt)
      PreArray   = [F*QP*DPsqrt, GQsqrt];
           [m,n] = size(PreArray);        
     if n<m, error('The matrix is fat, i.e. m>n'); end;

     [QP,DPsqrt] = svd(PreArray,'econ'); 
               X = F*X;                  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,DPsqrt,QP,residual,DRek,QRek] = svd_update(X,DPsqrt,QP,z,H,sqrtR)
     PreArray    = [H*QP*DPsqrt, sqrtR];
           [m,n] = size(PreArray);        
     if n<m, error('The matrix is fat, i.e. m>n'); end;

     [QRek,DRek_sqrt] = svd(PreArray,'econ');          % SVD factors of R_{e,k} 
           DRek = DRek_sqrt.^2;         
    
     residual  =  z-H*X;                               % residual
          Gain = QP*(DPsqrt^2)*QP'*H'*QRek/DRek*QRek'; % Kalman gain
             X = X + Gain*residual;                    % Filtered estimate 

       PreArray  = [(eye(size(QP,1)) - Gain*H)*QP*DPsqrt, Gain*sqrtR];  % SVD of P
     [QP,DPsqrt] = svd(PreArray,'econ'); 
end
