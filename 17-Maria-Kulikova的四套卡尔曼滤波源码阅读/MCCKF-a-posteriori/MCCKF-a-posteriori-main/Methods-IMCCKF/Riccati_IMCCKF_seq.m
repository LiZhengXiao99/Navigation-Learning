% ------------------------------------------------------------------- 
% Sequential Improved Maximum Correntropy Criterion Kalman Filter (IMCC-KF)
%      Type: Covariance filtering
%    Method: Conventional implementation
%      From: Two stages, a posteriori form, component-wise measurement update
% Recursion: Riccati-type underlying recursion
%   Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
% References:
% 1. This is the implementation of Algorithm 3 from the following paper:
%    Kulikova M.V. (2020) Sequential maximum correntropy Kalman filtering. 
%    Asian Journal of Control, 22(1):25-33.
%    DOI: https://doi.org/10.1002/asjc.1865
% 2. The calculation of the Baram PI is similar to the log LF computation proved in:
%    Kulikova M.V. (2003) 
%    On effective computation of the logarithm of the likelihood ratio 
%    function for Gaussian signals, Lecture Notes in Computer Science, 
%    2658: 427-435 DOI: http://doi.org/10.1007/3-540-44862-4_45
%
% ------------------------------------------------------------------- 
% Input:
%     matrices        - system matrices F,G, etc
%     initials_filter - initials x0,P0
%     measurements    - measurement history
% Output:
%     PI          - performance index (Baram Proximity Measure here)
%     hatX        - estimates (history) 
%     hatDP       - diag of the filtered error covariance (history)
% ------------------------------------------------------------------- 
function [PI,hatX,hatDP] = Riccati_IMCCKF_seq(matrices,initials_filter,measurements,handle_kernel)
   [F,G,Q,H,R] = deal(matrices{:});         % get system matrices
   GQG_product = G*Q*G';                    % compute once
         [X,P] = deal(initials_filter{:});  % initials for the filter 
   
        [m,n]  = size(H);                 % dimensions
       N_total = size(measurements,2);    % number of measurements
         hatX  = zeros(n,N_total+1);      % prelocate for efficiency
         hatDP = zeros(n,N_total+1);      % prelocate for efficiency
            PI = 0;                       % set initial value for the PI
 fl=0; 
 if isequal(diag(diag(R)),R)==0                              % if R does not have a diag form,
    [H,R,measurements,fl] = DecorrelateR(H,R,measurements);  % one needs a decorrelation procedure
 end; 

hatX(:,1) = X; hatDP(:,1) = diag(P);    % save initials at the first entry
for k = 1:N_total              
      [X,P]        = kf_predict(X,P,F,GQG_product); 
      lambda_k     = feval(handle_kernel,matrices,X,P,measurements(:,k));
      if (size(lambda_k,1)>1)||(size(lambda_k,2)>1), error('The IMCC-KF estimator implies a scalar adjusting parameter'); end;
      for i=1:m, 
         [X,P,ei,alph] = imcckf_update(X,P,measurements(i,k),H(i,:),R(i,i),lambda_k); 
      end;
      if fl==0, PI = PI + 1/2*log(abs(alph)) + 1/2*(ei^2)/alph;     % if measurement model is changed, then the LLF should be also
                 else PI = NaN; end;                                % transformed. For a fair comparison with other filters, we set the LLF to NaN.

      hatX(:,k+1)  = X; hatDP(:,k+1) = diag(P); % save  information
 end;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Time update: a priori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P] = kf_predict(X,P,F,GQG_product)
    X = F*X;                  % Predicted state estimate  
    P = F*P*F' + GQG_product; % Predicted error covariance 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement update: a posteriori estimates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,P,residual,cov_residual] = imcckf_update(X,P,z,H,R,lambda_k)
  residual     = z - H*X;            % residual
  cov_residual = R + lambda_k*H*P*H';         % residual covariance matrix 
  Kalman_gain  = lambda_k*P*H'/cov_residual;  % Gain matrix

  X = X + Kalman_gain*residual;      % Filtered state estimate
  P = P-Kalman_gain*H*P;             % Filtered error covariance
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%   Measurement de-correlation:  based on Cholesky d.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
