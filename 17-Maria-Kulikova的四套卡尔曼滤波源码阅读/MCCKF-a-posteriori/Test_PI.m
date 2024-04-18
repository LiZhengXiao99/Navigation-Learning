% ------------------------------------------------------------------- 
% Script for comparing various KF implementation methods and 
% the Performance Index evaluation (Baram Proximity Measure) 
% Authors: Maria Kulikova     
% ------------------------------------------------------------------- 

clear all; close all; clc; warning off;

% ---- Parameters that you may change ------
Delta    = 2;                  % Sampling interval Delta = t_{k}-t_{k-1}
N_total  = 120;                % Discrete-time instances
true_tau = 10;                 %'true' value for the system parameter
grid4tau = 3:1:35;             % grid for parameter value

% ----Load  Model to be examined ----
p = pwd; cd('Noises/'); noise_type = @noise_gauss; cd(p);                          % Type of uncertainties 
p = pwd; cd('Methods-Kernel/'); handle_kernel = @Gauss_weighted_const; cd(p);      % Adaptive kernel size selection strategy
p = pwd; cd('Models/'); [Fsym,Gsym,Qsym,Hsym,Rsym,P0sym,x0sym,parameters] = PModel_simple(Delta); cd(p); 

%--- Filtering methods to be examined ----
p = pwd; cd('Methods-MCCKF'); 
    % ----- Conventional algorithms --------
    handle_funs{1} = @Riccati_MCCKF;            % Original implementation by Izanloo et.al (2016)

   % ----- Cholesky-based methods --------------------
    handle_funs{2} = @Riccati_MCCKF_SRCF_QR;    % Square-Root Covariance Filter (SRCF) by Kulikova (2019), upper triangular factors
    handle_funs{3} = @Riccati_MCCKF_SRCF_QL;    % SRCF by Kulikova (2020), lower triangular factors
    handle_funs{4} = @Riccati_MCCKF_rSRCF_QL;   % robust SRCF by Kulikova (2020), lower triangular factors

   % ----- SVD-based methods --------------------
    handle_funs{5} = @Riccati_MCCKF_SVD;        % SVD-based mixed-type filter by Kulikova (2019)
    handle_funs{6} = @Riccati_MCCKF_rSVD;       % robust SVD-based covariance filter by Kulikova (2019)

cd(p); 

 Number_Methods = size(handle_funs,2);       % number of methods to be tested
 filters        = cell(Number_Methods,1);    % prelocate for efficiency

  % ----Simulate the system to get "true" state and measurements  ----
  % substitute the "True" parameter value 
  [F,G,Q,H,R,P0,x0]  = Substitute(parameters,true_tau,Fsym,Gsym,Qsym,Hsym,Rsym,P0sym,x0sym);
  [DT,EX,yk,~,~] = Simulate_Measurements(noise_type,{F,G,Q,H,R},{x0,P0},N_total);
      DST = [DT(:,1), DT(:,2:Delta:end)];                  % Discrete time points of sampling interval
      Exact_StateVectorX = [EX(:,1), EX(:,2:Delta:end)];   % Exact X at the points of sampling interval
      Measurements = yk(:,1:Delta:end);                    % Data come at the points of sampling intervals

 for exp_number = 1:length(grid4tau)  
     current_tau = grid4tau(exp_number);
     fprintf(1,'Simulation tau = %8.4f: \n',current_tau); 
     [Fi,Gi,Qi,Hi,Ri,P0i,x0i]    = Substitute(parameters,current_tau,Fsym,Gsym,Qsym,Hsym,Rsym,P0sym,x0sym);

     % ----Solve the inverse problem, i.e. perform the filtering process and compute PI
     for i=1:Number_Methods;
       filters{i}.legend = func2str(handle_funs{i}); 
       tstart = tic;
       [PI,hatX,hatDP] =  feval(handle_funs{i},{Fi,Gi,Qi,Hi,Ri},{x0i,P0i},Measurements,handle_kernel);
       ElapsedTime = toc(tstart);
       filters{i}.hatX(:,:,exp_number)  = hatX;
       filters{i}.hatDP(:,:,exp_number) = hatDP;
       filters{i}.trueX(:,:,exp_number) = Exact_StateVectorX;
       filters{i}.DST(:,:,exp_number)   = DST;
       filters{i}.yk(:,:,exp_number)    = Measurements;
       filters{i}.AE(:,:,exp_number)    = abs(hatX-Exact_StateVectorX); 
       filters{i}.Time(exp_number)      = ElapsedTime;
       filters{i}.PI(exp_number)        = PI;
       filters{i}.grid(exp_number)      = current_tau; % save the grid for parameters 

    end;
end;
fprintf(1,' DONE \n \n');
% Illustrate_XP(filters(1:Number_Methods));  % One may illustrate all filters or part of them  
 Illustrate_PI(filters);
