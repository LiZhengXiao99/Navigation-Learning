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
function [Times,ExactX,measurements] =  Simulate_Measurements(noise_type,matrices,initialX,N_total,start_mode)
[F,G,Q,H,R] = deal(matrices{:});  % get system matrices
[bar_x0,P0] = deal(initialX{:});  % get initials
      [~,q] = size(G);            % dimension of the process noise
      [m,n] = size(H);            % dimension of the measurement and state vectors
      Times = 0:N_total;          % time grid
        
  measurements = zeros(m,N_total+1); % prelocate for efficiency
  ExactX       = zeros(n,N_total+1); % prelocate for efficiency
 
  mean_noise_v = zeros(m,1);  % zero-mean noise  
  mean_noise_w = zeros(q,1);  % zero-mean noise 
 [noise_w] = feval(noise_type,mean_noise_w,Q,N_total+1);  % generate noise w 
 [noise_v] = feval(noise_type,mean_noise_v,R,N_total+1);  % generate noise v
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%          Initialization
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 if nargin<5, start_mode = true; end;
 if start_mode
    ExactX(:,1)  = bar_x0; % if we want to start from the exact x_0
 else
    ExactX(:,1) = noise_gauss(bar_x0,P0,1); % if x_0 is random
 end;

  % Note that there exists measurement at time t0=0.

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%        Simulation
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for k=1:N_total+1
     measurements(:,k) = H*ExactX(:,k) + noise_v(:,k);
     ExactX(:,k+1)     = F*ExactX(:,k) + G*noise_w(:,k);
  end;

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%        Final step
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % The first one corresponds to the time point t0 and we have 
 % N_total+1 time points. So, we delete the last one in X.
  ExactX(:,N_total+2) = []; 
end
