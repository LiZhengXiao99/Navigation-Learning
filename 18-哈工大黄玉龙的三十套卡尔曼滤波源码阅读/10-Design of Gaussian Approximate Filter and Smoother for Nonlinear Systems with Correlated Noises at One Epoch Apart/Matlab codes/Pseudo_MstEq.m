function z=Pseudo_MstEq(xk1,xk,S,Q)

G=S'*inv(Q);

z=MstEq(xk1)+G*(xk1-ProssEq(xk));