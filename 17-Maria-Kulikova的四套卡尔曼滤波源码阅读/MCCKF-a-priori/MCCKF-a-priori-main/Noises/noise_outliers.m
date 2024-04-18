% ------------------------------------------------------------------- 
% Function for generating the Gaussian noise with outliers.
% Authors: Maria Kulikova 
% ------------------------------------------------------------------- 
function [X] = noise_shot(M,S,N)
    % ---generate Gaussian uncertainties----   
    L = chol(S,'lower');  
    X = repmat(M,1,N) + L*randn(size(M,1),size(M,2)*N);

    % ---- Parameters that you may change ------
    percent_shots      = 0.3;                       % percent of the shots
    number_shots       = fix(percent_shots*N);      % number of shots
    start_of_shotnoise = 15;                        % the first few measurements are free of shots

    % ---randomly choose the indeces of measurements corrupted by shots---- 
    outlier_indices = unique(randi([start_of_shotnoise+1  N-1],1,number_shots)); 

    % --- Gaussian noise with outliers ---- 
    NN = length(outlier_indices);  
    X(:,outlier_indices) = repmat(M,1,NN) + 3*L*randn(size(M,1),size(M,2)*NN);
end
