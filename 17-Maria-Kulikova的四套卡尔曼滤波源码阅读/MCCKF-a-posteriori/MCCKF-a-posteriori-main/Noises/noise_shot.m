% ------------------------------------------------------------------- 
% Function for generating the shot noise (impulsive noise).
% Authors: Maria Kulikova:     maria dot kulikova at ist dot utl dot pt 
% License: GNU General Public License Version 2 
% ------------------------------------------------------------------- 
function [X] = noise_shot(M,S,N)
    % ---- Parameters that you may change ------
    percent_shots      = 0.2;                  % percent of the shots in yk
    number_shots       = fix(percent_shots*N); % number of shots
    magnitude_shot0    = 1;             % magnitude of the shot noise        
    magnitude_shotN    = 5;
    start_of_shotnoise = 15;            % the first few measurements are free of shots

    % ---randomly choose the indeces of measurements corrupted by shots---- 
    index_rand_shot    = unique(randi([start_of_shotnoise+1  N-1],1,number_shots)); 

    % ---generate Gaussian uncertainties----   
    L = chol(S,'lower'); 
    X = repmat(M,1,N) + L*randn(size(M,1),size(M,2)*N);
    
    % ---corrupt the Gaussian noise by random shots----   
    for i=1:length(index_rand_shot)
        temp = randi([magnitude_shot0  magnitude_shotN],size(S,1),1);
        X(:,index_rand_shot(i)) =  X(:,index_rand_shot(i)) + temp; 
   end;
end
