% ------------------------------------------------------------------- 
% Function for generating Gaussian uncertainties.
%   M - Dx1 mean of distibution
%   S - DxD covariance matrix
%   N - Number of samples
% ------------------------------------------------------------------- 
function [X] = noise_gauss(M,S,N)
  L = chol(S,'lower'); % should be lower triangular matrix
  X = repmat(M,1,N) + L*randn(size(M,1),size(M,2)*N);
end
  
