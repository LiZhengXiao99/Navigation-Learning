function [xkk,Pkk]=ivbkf(xkk,Pkk,F,H,z,Q,R,N,v1,v2)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
xkk=xk1k;

Pkk=Pk1k;

for i=1:N
    
    %%%%%%%Calculate auxiliary parameter 
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama1=trace(Dk1*inv(Pk1k));

    gama2=trace(Dk2*inv(R));

    lamda1=(v1+nx)/(v1+gama1);

    lamda2=(v2+nz)/(v2+gama2);
    
    %%%%%%%Update the distribution of state vector
    D_Pk1k=Pk1k/lamda1;

    D_R=R/lamda2;

    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
end



