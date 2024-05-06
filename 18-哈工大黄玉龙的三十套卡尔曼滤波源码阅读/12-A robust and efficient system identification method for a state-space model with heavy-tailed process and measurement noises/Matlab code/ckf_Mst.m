function z=ckf_Mst(d,x)

n=size(x,2); 

for i=1:n
    
    z(:,i)=MstEq(d,x(:,i));
    
end
