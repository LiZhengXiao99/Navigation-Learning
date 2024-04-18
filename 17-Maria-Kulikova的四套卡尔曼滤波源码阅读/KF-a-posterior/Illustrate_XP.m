% ------------------------------------------------------------------- 
% Script for illustrating the filtered estimates and the diagonal entries 
% of the error covariance matrix (over time). 
% Authors: Maria Kulikova:  kulikova dot maria at yahoo dot com     
% License: GNU General Public License Version 2 
% ------------------------------------------------------------------- 

function Illustrate_XP(out)
[N_state,DT,MC_runs] = size(out{1}.hatX);
time_grid = out{1}.DST(1,:,1);  
N_filters  = size(out,1);
marks = '.ox+*sdv^><ph';

 %%%% error covariance elements %%%%%%%%%%%%%
 for i=1:N_state
 figure;
 Filter_Set = []; %cell(1,N_filters);
 for j=1:N_filters
   if ~isempty(out{j}.hatDP) % if not empty  
     styles = [':' marks(round(12*rand+1))];
     mean_DP = mean(out{j}.hatDP,3);
     plot(time_grid,mean_DP(i,:),styles,'Color',[rand rand rand]);
     hold on;
     str1 = out{j}.legend; inds = find(str1=='_'); str1(inds)=' ';
     Filter_Set =  [Filter_Set  {str1}]; 
   end;
 end;
 legend(Filter_Set,1);
 title(['Fig.' num2str(i) ' Variance history var(' num2str(i) ') averaged over ' num2str(MC_runs) ' Monte Carlo runs']);
 xlabel('Discrete time'); ylabel('Variance');
 % grid on;  hold off;
end;

%%%%  states %%%%%%%%%%%%%
for i=1:N_state
 figure;
 Filter_Set = []; %cell(1,N_filters);
 for j=1:N_filters
   if ~isempty(out{j}.hatX) % if not empty    
     styles = [':' marks(round(12*rand+1))];  
     mean_X = mean(out{j}.hatX,3);          
     plot(time_grid,mean_X(i,:),styles,'Color',[rand rand rand]);
     hold on;
     str1 = out{j}.legend; inds = find(str1=='_'); str1(inds)=' ';
     Filter_Set =  [Filter_Set  {str1}]; 
   end;  
 end;

 legend(Filter_Set,1);
 title(['Fig.' num2str(i) ' State vector x(' num2str(i) ') averaged over ' num2str(MC_runs) ' Monte Carlo runs']);
 xlabel('Discrete time'); ylabel('State vector component');
 % grid on;   hold off;
end;
end
