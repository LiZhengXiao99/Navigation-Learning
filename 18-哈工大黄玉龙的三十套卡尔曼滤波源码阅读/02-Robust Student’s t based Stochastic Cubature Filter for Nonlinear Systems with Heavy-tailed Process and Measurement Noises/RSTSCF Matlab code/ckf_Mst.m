function z=ckf_Mst(x,xp,yp)

n = size(x,2); 

for i=1:n
    
    z(:,i)=MstEq(x(:,i),xp,yp);
    
end
