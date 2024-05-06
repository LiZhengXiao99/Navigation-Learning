function [max_delta]=adaptive_delta(xk1k1,Pk1k1,z,Q,R,step,t)

%%%%Preparation
max_delta=0;  %%%%%%%%%%%%%Initial value for delta

max_LH=0;     %%%%%%%%%%%%%Initial value for maximum likelihood

nx=size(xk1k1,1);

nPts=2^nx;  

Sk1k1=utchol(Pk1k1);

for delta=sqrt(1/2):step:sqrt(3/2)
    
    CPtArray=embedded_sigma_point(delta); 

    w0=1-1/(2*delta^2);

    w1=1/(2^(nx+1)*delta^2);
    
    %%%%Time-update
    Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;
    
    Xk1k1=[xk1k1 Xk1k1];
    
    Xkk1=ckf_ProssEq(Xk1k1);  

    xkk1= w0*Xkk1(:,1)+w1*sum(Xkk1(:,2:end),2);

    Pkk1=w0*Xkk1(:,1)*Xkk1(:,1)'+w1*Xkk1(:,2:end)*Xkk1(:,2:end)'-xkk1*xkk1'+Q;    
    
    %%%%Measurement-update
    Skk1=utchol(Pkk1);
    
    Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;

    Xi=[xkk1 Xi];

    Zi=ckf_Mst(Xi,t);

    zkk1=w0*Zi(:,1)+w1*sum(Zi(:,2:end),2);

    Pzzkk1=w0*Zi(:,1)*Zi(:,1)'+w1*Zi(:,2:end)*Zi(:,2:end)'-zkk1*zkk1'+R;  

    LH=(1/sqrt(det(2*pi*Pzzkk1)))*exp(-0.5*(z-zkk1)'*inv(Pzzkk1)*(z-zkk1));  %%%%%%Likelihood function value
    
    if LH>=max_LH
        
        max_delta=delta;
        
        max_LH=LH;
        
    end
    
end
