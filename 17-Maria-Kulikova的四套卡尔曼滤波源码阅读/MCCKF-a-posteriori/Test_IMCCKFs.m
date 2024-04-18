% ------------------------------------------------------------------- 
% Script for comparing various IMCC-KF implementation methods.
% Implementation: Maria Kulikova     
% ------------------------------------------------------------------- 

clear all; close all; clc; warning off;

% ---- Parameters that you may change ------
MC_runs  = 10;                 % Number of Monte Carlo runs
Delta    = 1;                  % Sampling interval Delta = t_{k}-t_{k-1}
N_total  = 100;                % Discrete-time instances

% ----Load  Model to be examined ----
p = pwd; cd('Noises/'); noise_type = @noise_outliers; cd(p);                        % Type of uncertainties 
p = pwd; cd('Methods-Kernel/'); handle_kernel = @Gauss_weighted_const; cd(p);       % Adaptive kernel size selection strategy
p = pwd; cd('Models/'); [Fsys,Gsys,Qsys,Hsys,Rsys,P0,x0] = Model_satellite; cd(p);  % Model to examine

%--- Filtering methods to be examined ----
p = pwd; cd('Methods-IMCCKF'); 
    % ----- Conventional algorithms --------
    handle_funs{1} = @Riccati_IMCCKF;            % conventional implementation by Kulikova (2017)
    handle_funs{2} = @Riccati_IMCCKF_seq;        % Sequential (component-wise measurement update) method by Kulikova (2020)

   % ----- Cholesky-based methods --------------------
    handle_funs{3} = @Riccati_IMCCKF_SRCF_QR;    % Square-Root Covariance Filter (SRCF) by Kulikova (2017), upper triangular factors
    handle_funs{4} = @Riccati_IMCCKF_eSRCF_QR;   % Extended SRCF by Kulikova (2017), upper triangular factors

   % ----- SVD-based methods --------------------
    handle_funs{5} = @Riccati_IMCCKF_SVD;        % SVD-based mixed-type filter by Kulikova (2019)

cd(p); 

 Number_Methods = size(handle_funs,2);       % number of methods to be tested
 filters        = cell(Number_Methods,1);    % prelocate for efficiency

% --- Monte Carlo runs ----

 for exp_number = 1:MC_runs  
     fprintf(1,'Simulation #%d: \n',exp_number); 

     % ----Simulate the system to get "true" state and measurements  ----
     [DT,EX,yk,Qsys,Rsys] = Simulate_Measurements(noise_type,{Fsys,Gsys,Qsys,Hsys,Rsys},{x0,P0},N_total);
      DST = [DT(:,1), DT(:,2:Delta:end)];                  % Discrete time points of sampling interval
      Exact_StateVectorX = [EX(:,1), EX(:,2:Delta:end)];   % Exact X at the points of sampling interval
      Measurements = yk(:,1:Delta:end);                    % Data come at the points of sampling intervals

     % ----Solve the inverse problem, i.e. perform the filtering process
     for i=1:Number_Methods;
       filters{i}.legend = func2str(handle_funs{i}); 
       tstart = tic;
       [PI,hatX,hatDP] = feval(handle_funs{i},{Fsys,Gsys,Qsys,Hsys,Rsys},{x0,P0},Measurements,handle_kernel);
       ElapsedTime = toc(tstart);
       filters{i}.hatX(:,:,exp_number)  = hatX;
       filters{i}.hatDP(:,:,exp_number) = hatDP;
       filters{i}.trueX(:,:,exp_number) = Exact_StateVectorX;
       filters{i}.DST(:,:,exp_number)   = DST;
       filters{i}.yk(:,:,exp_number)    = Measurements;
       filters{i}.AE(:,:,exp_number)    = abs(hatX-Exact_StateVectorX); 
       filters{i}.Time(exp_number)      = ElapsedTime;
       filters{i}.PI(exp_number)        = PI;
    end;
end;

% --- Estimation errors computation  ----
for i=1:Number_Methods;
   val1 = mean((filters{i}.AE).^2,3);             % mean by Monte Carlo runs 
   filters{i}.RMSE = sqrt(mean(val1,2));          % mean by time, i.e. RMSE in each component
end;

% --- Print the results  
fprintf(1,'--------------------- \n'); fprintf(1,'  Filter Implementations:\t');       
for i=1:size(P0,1), fprintf(1,'RMSE_x(%d)\t',i); end; fprintf(1,'||RMSE||_2 \t av.CPU (s) \t PI \n');
for i=1:Number_Methods
 fprintf(1,'%d.%22s\t ',i,filters{i}.legend); 
 fprintf(1,'%8.4f\t',filters{i}.RMSE); fprintf(1,'%8.4f\t%8.4f\t%8.4f \n',norm(filters{i}.RMSE,2),mean(filters{i}.Time),filters{i}.PI(1));
end;

% --- Illustrate, if you wish (ensure that all related entries coincide)
% Illustrate_XP(filters(1:Number_Methods));  % One may illustrate all filters or part of them  


