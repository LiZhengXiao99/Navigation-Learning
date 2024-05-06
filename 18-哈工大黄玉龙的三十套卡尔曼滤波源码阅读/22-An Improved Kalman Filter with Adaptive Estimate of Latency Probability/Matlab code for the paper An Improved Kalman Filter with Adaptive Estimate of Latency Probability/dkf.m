function [xak1k1,Pak1k1] = dkf(xakk,Pa,F,H,y,Q,R,p)

nx=size(Q,1);

nz=size(R,1);

%%%%%%%%Time update
xkk=xakk(1:nx,1);

Pkk=Pa(1:nx,1:nx);

xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%%%%One step prediction of ideal measurement
zk1k =H*xk1k;   

Pzzk1k=H*Pk1k*H'+R;

Pxzk1k=Pk1k*H';

zkk=H*xkk;

Pzzkk=H*Pkk*H'+R;

Pxzk1kk=F*Pkk*H';

%%%%%%%%One step prediction of actual measurement
yk1k=(1-p)*zk1k+p*zkk;

Pyyk1k=(1-p)*Pzzk1k+p*Pzzkk+p*(1-p)*(zk1k-zkk)*(zk1k-zkk)';

Pxyk1k=(1-p)*Pxzk1k+p*Pxzk1kk;

%%%%%%%%Filter eastimation of state
Wk=Pxyk1k*inv(Pyyk1k);     

xk1k1=xk1k+Wk*(y-yk1k);

Pk1k1=Pk1k-Wk*Pyyk1k*Wk';

vk1k1=(1-p)*R*inv(Pyyk1k)*(y-yk1k);

Pvvk1k1=R-(1-p)^2*R*inv(Pyyk1k)*R;

Pxvk1k1=-(1-p)*Pxyk1k*inv(Pyyk1k)*R;

xak1k1=[xk1k1;vk1k1];

Pak1k1=[Pk1k1 Pxvk1k1;Pxvk1k1' Pvvk1k1];

end
  