function z=ckf_Mst(x)

n=size(x(:,1),1); 

for i=1:2*n
    
    z(:,i)=MstEq(x(:,i));
    
end