function z=ckf_Mst(x)

n=size(x,2); 

for i=1:n
    
    z(:,i)=MstEq(x(:,i));
    
end
