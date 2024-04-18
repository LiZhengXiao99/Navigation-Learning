% ------------------------------------------------------------------- 
% Function for generating Gaussian uncertainties.
%   M - Dx1 mean of distibution
%   S - DxD covariance matrix
%   N - Number of samples
% ------------------------------------------------------------------- 
function [X] = noise_gauss(M,S,N)
  if isequal(diag(diag(S)), S), % if diag form
     L = diag(sqrt(diag(S))); 
  else  
     L = chol(S,'lower'); 
  end; 
  X = repmat(M,1,N) + L*randn(size(M,1),size(M,2)*N);
end
  

