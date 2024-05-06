function y=SGGM(x,u,sigma,nu,beta,M)

n=size(x,2);

y=zeros(1,n);

Y=gamrnd(nu/2,2/nu,1,M);

for i=1:n
    
    xi=x(i);
    
    y(i)=Integral_MC(xi,u,sigma,beta,Y,M);

end

end