function [xk1k1,Pk1k1] = ikf(xkk,Pkk,F,H,z,Q,R,rk0)

%%%%%%%%% Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%%%%% Received measurement
if rk0==1
    
    zk1k=H*xk1k;
    
    Pzzk1k=H*Pk1k*H'+R;
    
    %%%%%%%%% Measurement update
    Pxzk1k=Pk1k*H';
    
    Wk=Pxzk1k*inv(Pzzk1k);
    
    xk1k1=xk1k+Wk*(z-zk1k);
    
    Pk1k1=Pk1k-Wk*Pzzk1k*Wk';
    
end

%%%%%%%%% Measurement loss
if rk0==0
    
    xk1k1=xk1k;
    
    Pk1k1=Pk1k;
    
end