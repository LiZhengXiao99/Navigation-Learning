function z=Pseudo_ckf_Mst(xk1,xk,S,Q)

n=size(xk1,2); 

for i=1:n
    
    z(:,i)=Pseudo_MstEq(xk1(:,i),xk(:,i),S,Q);
    
end
