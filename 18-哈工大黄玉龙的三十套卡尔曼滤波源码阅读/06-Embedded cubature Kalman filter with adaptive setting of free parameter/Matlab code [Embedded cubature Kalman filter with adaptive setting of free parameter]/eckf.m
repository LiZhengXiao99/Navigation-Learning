function [xkk,Pkk]=eckf(xk1k1,Pk1k1,z,Q,R,delta,t)

%%%%Preparation
nx=size(xk1k1,1);   

nPts=2^nx;  

CPtArray=embedded_sigma_point(delta); 

w0=1-1/(2*delta^2);

w1=1/(2^(nx+1)*delta^2);

%%%%Time-update
Sk1k1=utchol(Pk1k1);

Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xk1k1=[xk1k1 Xk1k1];

Xkk1=ckf_ProssEq(Xk1k1);  

xkk1=w0*Xkk1(:,1)+w1*sum(Xkk1(:,2:end),2);

Pkk1=w0*Xkk1(:,1)*Xkk1(:,1)'+w1*Xkk1(:,2:end)*Xkk1(:,2:end)'-xkk1*xkk1'+Q;    

%%%%Measurement-update
Skk1=utchol(Pkk1);

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;

Xi=[xkk1 Xi];

Zi=ckf_Mst(Xi,t);

zkk1=w0*Zi(:,1)+w1*sum(Zi(:,2:end),2);

Pzzkk1=w0*Zi(:,1)*Zi(:,1)'+w1*Zi(:,2:end)*Zi(:,2:end)'-zkk1*zkk1'+R;  

Pxzkk1=w0*Xi(:,1)*Zi(:,1)'+w1*Xi(:,2:end)*Zi(:,2:end)'-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);   

xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';  
