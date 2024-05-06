function z=ckf_Mst(x,t)

n = size(x,2); 

for i=1:n
    
    z(:,i)=MstEq(x(:,i),t);
    
end
