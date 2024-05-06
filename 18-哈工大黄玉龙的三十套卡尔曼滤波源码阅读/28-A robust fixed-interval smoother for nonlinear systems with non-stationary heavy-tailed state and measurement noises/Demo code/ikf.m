function [xk1k1,Pk1k1,xk1k,Pk1k,Pkk1k]=ikf(xkk,Pkk,F,H,z,Q,R)

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

Pk1k=(Pk1k+Pk1k')/2;        %%%%%%%%%%%%Keep Symmetry

Pkk1k=Pkk*F';

%%%%%Measurement update
Pzzk1k=H*Pk1k*H'+R;

Pxzk1k=Pk1k*H';

Kk=Pxzk1k*inv(Pzzk1k);

xk1k1=xk1k+Kk*(z-H*xk1k);

Pk1k1=Pk1k-Kk*H*Pk1k;