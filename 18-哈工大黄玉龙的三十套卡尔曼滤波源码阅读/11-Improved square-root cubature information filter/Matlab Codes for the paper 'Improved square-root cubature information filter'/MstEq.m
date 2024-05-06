function y=MstEq(x,Xs,Ys)

y=[];

ns=size(Xs,2);

for s=1:ns
    
    y_s=MstEq_sigle(x,Xs(s),Ys(s));

    y=[y y_s];
    
end



