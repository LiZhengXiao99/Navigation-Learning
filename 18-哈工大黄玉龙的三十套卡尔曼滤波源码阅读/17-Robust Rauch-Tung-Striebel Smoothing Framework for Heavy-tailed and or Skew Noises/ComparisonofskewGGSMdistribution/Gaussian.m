function y=Gaussian(x,u,sigma)

y0=1/sqrt(2*pi*sigma);

y1=exp((-0.5/sigma)*(x-u).^2);

y=y0*y1;