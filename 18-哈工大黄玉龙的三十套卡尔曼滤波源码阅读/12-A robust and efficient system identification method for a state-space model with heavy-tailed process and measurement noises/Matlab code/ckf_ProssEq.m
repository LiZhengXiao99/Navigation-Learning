function x=ckf_ProssEq(a,b,x,t)

n=size(x,2); 

for i=1:n
    
    x(:,i)=ProssEq(a,b,x(:,i),t);
    
end