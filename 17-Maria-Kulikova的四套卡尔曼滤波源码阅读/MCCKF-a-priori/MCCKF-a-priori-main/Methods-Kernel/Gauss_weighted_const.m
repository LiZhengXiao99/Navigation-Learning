% ------------------------------------------------------------------- 
% original Maximum Correntropy Criterion Kalman Filter (MCC-KF) kernel size selection
% Type: Gaussian kernel
%
% References:
% 1. Adjusting parameter lambda in Algorithm 2 in this paper
%    R. Izanloo, S. A. Fakoorian, H. S. Yazdi, D. Simon, Kalman filtering
%    based on the maximum correntropy criterion in the presence of non-
%    Gaussian noise, in: 2016 Annual Conference on Information Science and
%    Systems (CISS), 2016, pp. 500-505.
%    DOI:  https://doi.org/10.1109/CISS.2016.7460553 
%
% 2. Codes were also available at http://embeddedlab.csuohio.edu/Correntropy
% ------------------------------------------------------------------- 
function [lambda_k] = Gauss_weighted_const(matrices,X,P,z);
  [F,G,Q,H,R] = deal(matrices{:});              % get system matrices
           ek = z-H*X;                          % residual or innovation term
       w_norm = sqrt((ek'/R)*ek);               % weighted norm with respect to matrix 
        sigma = w_norm;                         % suggested in the codes [2] accompanying paper [1]

  G_denominator = exp(-0/(2*sigma^2));           % see explanation after formula (17) in [1] and codes [2];
  G_numerator   = exp(-(w_norm^2)/(2*sigma^2));  % Gaussian kernel after formula (12) in [1]; see also formula (28) in [1]

  lambda_k = G_numerator/G_denominator;          % see Algorithm 2 in [1]
end
