% ------------------------------------------------------------------- 
% Script for illustrating the Performance Index (Baram Proximity Measure) 
% Implementation: Maria Kulikova      
% ------------------------------------------------------------------- 

function Illustrate_PI(out)
N_filters  = size(out,1);
marks = '.ox+*sdv^><ph';

%%%%  states %%%%%%%%%%%%%
 figure;
 Filter_Set = []; %cell(1,N_filters);
 for j=1:N_filters
   if ~isempty(out{j}.PI) % if not empty    
     styles = [':' marks(round(12*rand+1))];  
     plot(out{j}.grid,out{j}.PI,styles,'Color',[rand rand rand]);
     hold on;
     str1 = out{j}.legend; inds = find(str1=='_'); str1(inds)=' ';
     Filter_Set =  [Filter_Set  {str1}]; 
   end;  
 end;
 legend(Filter_Set,1);

 arr = get(gca,'XTick');
 title('Fig.1 Performance Index (Baram Proximity Measure)');
 xlabel('Parameters Values');
 ylabel('Performance Index');
 %set(gca,'XLim',[out{1}.grid(1) out{1}.grid(end)]);
 grid on;   hold off;
end


