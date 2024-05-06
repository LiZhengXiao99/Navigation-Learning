function z=ckf_Mst(x,Xs,Ys)

z=[];

n = size(x,2); 

ns=size(Xs,2);

for s=1:ns
    
    for i=1:n
        
        y=MstEq_sigle(x(:,i),Xs(s),Ys(s));
        
        z=[z y];
        
    end
    
end
