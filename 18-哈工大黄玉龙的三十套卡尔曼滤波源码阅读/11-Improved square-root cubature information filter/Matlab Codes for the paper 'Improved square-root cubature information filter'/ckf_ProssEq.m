function x=ckf_ProssEq(x,T)

n = size(x,2); 

for i=1:n
    
    x(:,i)=ProssEq(x(:,i),T);
    
end