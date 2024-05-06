function y=MstEq_new(x,phi)

n=size(x,1)/2;

xk1=x(1:n);

xk=x(n+1:end);

y=MstEq(xk1)-phi*MstEq(xk);
