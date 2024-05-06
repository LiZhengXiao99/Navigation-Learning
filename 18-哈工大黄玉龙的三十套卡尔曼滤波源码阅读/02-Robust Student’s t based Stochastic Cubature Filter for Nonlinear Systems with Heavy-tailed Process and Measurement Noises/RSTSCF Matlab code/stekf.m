function [xkk,Skk] = stekf(xkk,Skk,z,Q,R,v,xp,yp)

%%%%%%
Skk=(v-2)/v*Skk;

nz=size(z,1);

%%%%%%%%%
F=Jacobian_f(xkk);

xkk1=ProssEq(xkk);

Pkk1=F*Skk*F'+Q;

%%%%%%%%%
H=Jacobian_h(xkk1,xp,yp);

zkk1=MstEq(xkk1,xp,yp);

Pzzkk1=H*Pkk1*H'+R;

Pxzkk1=Pkk1*H';

%%%%%%%
Wk=Pxzkk1*inv(Pzzkk1);        

xkk=xkk1+Wk*(z-zkk1);

deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);

Skk=(v+deta2)/(v+nz)*(Pkk1-Wk*Pzzkk1*Wk');

vvv=v+nz;

%%%%%%%
Skk=vvv/(vvv-2)*Skk;

