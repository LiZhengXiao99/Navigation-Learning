function [xkk,Pkk]=stkf(xkk,Pkk,F,H,z,Q,R,v)

nx=size(xkk,1);

nz=size(z,1);

%%%%%%%%%Time update
xkk1=F*xkk;

Pkk1=F*((v-2)/v*Pkk)*F'+Q;

%%%%%%%%%Measurement update
zkk1=H*xkk1;

Pzzkk1=H*Pkk1*H'+R;

Pxzkk1=Pkk1*H';

%%%%%%%Initial update
Wk=Pxzkk1*inv(Pzzkk1);        

xkk=xkk1+Wk*(z-zkk1);

deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);

Pkk=(v+deta2)/(v+nz)*(Pkk1-Wk*Pzzkk1*Wk');

vvv=v+nz;

%%%%%%%Final update
Pkk=(vvv/(vvv-2))*Pkk;
