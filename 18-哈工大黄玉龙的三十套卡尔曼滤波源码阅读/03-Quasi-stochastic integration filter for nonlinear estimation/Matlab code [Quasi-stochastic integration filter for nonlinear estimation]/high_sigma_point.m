function [sigma]=high_sigma_point(nx,k2)

%%%%Preparation
I=eye(nx);

s1=sqrt((4-nx)*(nx+k2)/(k2+2-nx));

s2=sqrt((nx+k2));

%%%%Sigma point sampling
%%%%sj+ and sj-
sj1=[];

sj2=[];

sj3=eye(nx);

for k=1:(nx-1)
    
    for m=(k+1):nx
        
        e1=sqrt(0.5)*(I(:,k)+I(:,m));
        
        e2=sqrt(0.5)*(I(:,k)-I(:,m));
        
        sj1=[sj1,e1];
        
        sj2=[sj2,e2];
        
    end
end

sigma1=s2*sj1;

sigma2=s2*sj2;

sigma3=s1*sj3;

sigma=[zeros(nx,1),sigma1,-sigma1,sigma2,-sigma2,sigma3,-sigma3];




