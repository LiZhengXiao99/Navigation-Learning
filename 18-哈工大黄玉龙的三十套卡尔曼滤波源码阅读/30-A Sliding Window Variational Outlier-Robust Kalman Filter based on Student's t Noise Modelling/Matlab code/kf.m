function [xkk,Pkk,xk1k,Pk1k]=kf(xkk,Pkk,F,H,z,Q,R)

%%%%%Time Update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement Update
Pzzk1k=H*Pk1k*H'+R;

Pxzk1k=Pk1k*H';

Kk=Pxzk1k*inv(Pzzk1k);

xkk=xk1k+Kk*(z-H*xk1k);

Pkk=Pk1k-Kk*H*Pk1k;