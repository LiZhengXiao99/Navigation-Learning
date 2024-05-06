function z=ckf_Mst_new(x,phi)

n=size(x,2); 

for i=1:n
    
    z(:,i)=MstEq_new(x(:,i),phi);
    
end