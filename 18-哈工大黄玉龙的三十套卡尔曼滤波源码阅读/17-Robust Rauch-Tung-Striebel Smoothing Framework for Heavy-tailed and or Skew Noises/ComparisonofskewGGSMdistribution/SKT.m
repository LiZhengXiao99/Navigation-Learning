function y=SKT(x,nu,beta)

n=size(x,2);

y=zeros(1,n);

if beta==0
    
    for i=1:n
        
        xi=x(i);

        y(i)=tpdf(xi,nu);
        
    end
    
else
    
    for i=1:n
        
        xi=x(i);
        
        cx=beta*xi*sqrt((nu+1)/(xi^2+nu));
        
        y0=2*tcdf(cx,nu+1);
        
        y1=tpdf(xi,nu);
        
        y(i)=y0*y1;
    
    end
    
end

