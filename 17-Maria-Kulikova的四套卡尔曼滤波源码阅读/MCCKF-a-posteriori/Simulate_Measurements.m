% Linear Discrete-time model simulation
%
% Input:
%     matrices     - system matrices F,G,Q,H,R etc.
%     N_total      - number of measuremens to generate
%     P0,bar_x0    - Initials
%     start_mode   - is 'true' if we want to start the simulation from the
%                    exact bar_x0; otherwise we start from random N(bar_x0,P0)
%                    Default is true 
% Output:
%     Time         - discrete-time grid
%     ExactX       - stochastic state vector;
%     measurements - system measurements;
%_____________________________________________________________________________________
function [Times,ExactX,measurements,Qnew,Rnew] =  Simulate_Measurements(noise_type,matrices,initialX,N_total,start_mode)
[F,G,Q,H,R] = deal(matrices{:});  % get system matrices
[bar_x0,P0] = deal(initialX{:});  % get initials
      [~,q] = size(G);            % dimension of the process noise
      [m,n] = size(H);            % dimension of the measurement and state vectors
      Times = 0:N_total;          % time grid
        
  measurements = zeros(m,N_total+1); % prelocate for efficiency
  ExactX       = zeros(n,N_total+1); % prelocate for efficiency
 
  mean_noise_v = zeros(m,1);  % zero-mean noise  
  mean_noise_w = zeros(q,1);  % zero-mean noise 
 [noise_w] = feval(noise_type,mean_noise_w,Q,N_total);  % generate noise w 
 [noise_v] = feval(noise_type,mean_noise_v,R,N_total);  % generate noise v
 Rnew = cov(noise_v');
 Qnew = cov(noise_w');
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%          Initialization
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 if nargin<5, start_mode = true; end;
 if start_mode
    ExactX(:,1)  = bar_x0; % if we want to start from the exact x_0
 else
    ExactX(:,1) = noise_gauss(bar_x0,P0,1); % if x_0 is random
 end;

  % Note that there is no measurements at time t0=0. Hence, we get 
  % the first measurement at t1 = t0 + h_t. 
  % Since X(1)=x0, we have a loop from 2.

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%        Simulation
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for k=2:N_total+1
     ExactX(:,k)       = F*ExactX(:,k-1) + G*noise_w(:,k-1);
     measurements(:,k) = H*ExactX(:,k)   +   noise_v(:,k-1);
  end;

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%        Final step
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % The first one corresponds to the time point t0, where we 
 % do not have measurements. So, we delete it.
  measurements(:,1) = []; 
end
