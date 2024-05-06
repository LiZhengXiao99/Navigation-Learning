% ------------------------------------------------------------------- 
% Maximum Correntropy Criterion Kalman Filter (MCC-KF)
%         Method: Conventional implementation
%           Type: Covariance filtering
%      Recursion: Riccati underlying recursion
%           Form: One stage (condensed), a priori form
%         Author: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
%    A priori form of the MCC-KF. The derivation can be found in 
% 1. Kulikova M.V. (2019) 
%    One-Step Condensed Forms for Square-Root Maximum Correntropy Criterion 
%    Kalman Filtering, Proceedings of the 23rd International Conference on
%    System Theory, Control and Computing (ICSTCC),  Sinaia, Romania, pp. 13-18. 
%    Oct. 2019,  DOI: http://doi.org/10.1109/ICSTCC.2019.8885950
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,H,Q etc   系统模型：过程噪声 Q、设计矩阵 H、状态转移矩阵 F
%     initials_filter - initials x0,P0              初始状态向量 x0、协方差 p0
%     measurements    - measurements (where y(t_k) is the k-th column) 模拟量测向量
% Output:
%     PI          - performance index (Baram Proximity Measure here) 性能指数
%     predX       - a priori estimates (history)                     预测状态向量
%     predDP      - diag of the predicted error covariance (history) 预测协方差
% ------------------------------------------------------------------- 
function [PI,predX,predDP] = Riccati_MCCKF_standard(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices          系统模型矩阵
         [X,P] = deal(initials_filter{:});  % get initials for the filter  系统初值
   
        [m,n]  = size(H);                 % get dimensions                 设计矩阵维数 m 行 n 列
       N_total = size(measurements,2);    % number of measurements         量测数
         predX = zeros(n,N_total+1);      % prelocate for efficiency       分配状态向量空间
        predDP = zeros(n,N_total+1);      % prelocate for efficiency       分配设协方差阵空间
            PI = 0;                       % set initial value for the PI   初始性能指数

 predX(:,1)  = X; predDP(:,1) = diag(P);  % save initials at the first entry
 
for k = 1:N_total       % 循环遍历各历元的量测，执行滤波过程
      lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
      if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The MCC-KF estimator implies a scalar adjusting parameter'); end;
      ek  = measurements(:,k)- H*X;        % residual
      Rek_lambda  = R + lambda_k*H*P*H';   % residual covariance matrix 

      Kpk_lambda  = lambda_k*F*P*H'/Rek_lambda;   % Gain matrix
      X = F*X + Kpk_lambda*ek;                    % Predicted state estimate  

      P = F*P*F' + G*Q*G' - Kpk_lambda*((2/lambda_k -1)*R + H*P*H')*Kpk_lambda';  % Predicted error covariance 
        
    PI = PI + 1/2*log(det(Rek_lambda))+1/2*ek'/Rek_lambda*ek; 
    predX(:,k+1) = X; predDP(:,k+1) = diag(P); 
 end;
end
