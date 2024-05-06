function y=Integral_MC(xi,u,sigma,beta,Y,M)

y=0;

for i=1:M
    
    Yi=Y(i);
    
    y=y+Gaussian(xi,u+beta/Yi,sigma/Yi);

end

y=y/M;

end

