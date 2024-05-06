function [sigma]=interpolatory_sigma_point(n,lamda_1)

I=eye(n);

sigma_1=[];

sigma_2=[];

for i=1:n-1   
    
    for j=i+1:n
        
        e1=lamda_1*(I(:,i)+I(:,j));
        
        e2=lamda_1*(I(:,i)-I(:,j));
        
        sigma_1=[sigma_1 e1];
        
        sigma_2=[sigma_2 e2];

    end 
    
end

sigma=[sigma_1 -sigma_1 sigma_2 -sigma_2];

