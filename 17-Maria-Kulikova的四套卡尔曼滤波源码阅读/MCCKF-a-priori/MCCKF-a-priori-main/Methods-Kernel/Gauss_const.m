% ------------------------------------------------------------------- 
% Kernel size selection from C-Filter published in [1]
% Type: Gaussian kernel
%
% References:
% 1. G. T. Cinar and J. C. Principe, Hidden state estimation using the
%    Correntropy Filter with fixed point update and adaptive kernel size,
%    IEEE World Congress on Computational Intelligence, Brisbane,
%    Australia, pp. 1-6, 2012.
%
% 2. R. Izanloo, S. A. Fakoorian, H. S. Yazdi, D. Simon, Kalman filtering
%    based on the maximum correntropy criterion in the presence of non-
%    Gaussian noise, in: 2016 Annual Conference on Information Science and
%    Systems (CISS), 2016, pp. 500-505.
%    DOI:  https://doi.org/10.1109/CISS.2016.7460553 
%
% 3. Codes were also available at http://embeddedlab.csuohio.edu/Correntropy
% ------------------------------------------------------------------- 
function [lambda_k] = Gauss_const(matrices,X,P,z);
  [F,G,Q,H,R] = deal(matrices{:});              % get system matrices
       w_norm = norm(z-H*X,2);                  % see C-Filter and suggestion in [1] 
        sigma = w_norm;                         % suggested in the codes [3] accompanying paper [2]

  G_denominator = exp(-0/(2*sigma^2));           % see explanation after formula (17) in [2] and codes [3];
  G_numerator   = exp(-(w_norm^2)/(2*sigma^2));  % Gaussian kernel after formula (12) in [2]; see also formula (28) in [3]

  lambda_k = G_numerator/G_denominator;          % see formula (19) in [2] for gain computaiton
end
