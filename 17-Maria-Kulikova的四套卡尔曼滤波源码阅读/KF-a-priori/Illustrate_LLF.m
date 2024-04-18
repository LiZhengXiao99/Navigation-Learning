% ------------------------------------------------------------------- 
% Script for illustrating the negative log likelihood function. 
% Authors: Maria Kulikova:  kulikova dot maria at yahoo dot com     
% ------------------------------------------------------------------- 

function Illustrate_LLF(out)
N_filters  = size(out,1);
marks = '.ox+*sdv^><ph';

%%%%  states %%%%%%%%%%%%%
 figure;
 Filter_Set = []; %cell(1,N_filters);
 for j=1:N_filters
   if ~isempty(out{j}.neg_LLF) % if not empty    
     styles = [':' marks(round(12*rand+1))];  
     plot(out{j}.grid,out{j}.neg_LLF,styles,'Color',[rand rand rand]);
     hold on;
     str1 = out{j}.legend; inds = find(str1=='_'); str1(inds)=' ';
     Filter_Set =  [Filter_Set  {str1}]; 
   end;  
 end;
 legend(Filter_Set,1);

 arr = get(gca,'XTick');
 title('Fig.1  Negative Log LF');
 xlabel('Parameters Values');
 ylabel('Negative Log Likelihood Function');
 %set(gca,'XLim',[out{1}.grid(1) out{1}.grid(end)]);
 grid on;   hold off;
end


