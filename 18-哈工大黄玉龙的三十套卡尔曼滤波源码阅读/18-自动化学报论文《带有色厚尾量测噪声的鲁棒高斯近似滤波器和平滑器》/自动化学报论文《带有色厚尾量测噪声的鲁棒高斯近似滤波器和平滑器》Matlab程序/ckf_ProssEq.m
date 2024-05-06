function x=ckf_ProssEq(x)

n=size(x,2); 

for i=1:n
    
    x(:,i)=ProssEq(x(:,i));
    
end