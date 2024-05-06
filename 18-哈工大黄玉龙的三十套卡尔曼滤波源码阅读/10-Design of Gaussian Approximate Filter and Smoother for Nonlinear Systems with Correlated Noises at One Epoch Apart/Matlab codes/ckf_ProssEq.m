function x=ckf_ProssEq(x)

n = size(x,1); 

for i=1:2*n
    
    x(:,i)=ProssEq(x(:,i));
    
end