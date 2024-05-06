function [xkk,Pkk]=high_eckf(xk1k1,Pk1k1,z,Q,R,delta,t)

%%%%Preparation
nx=size(xk1k1,1);   

nPts1=2*nx;  

nPts2=2^nx;  

CPtArray1=sqrt((4*delta^2)/(2*delta^2-1))*[eye(nx) -eye(nx)];

CPtArray2=embedded_sigma_point(delta);

w0=-(2*delta^2-1)*(2*nx*delta^2-4*delta^2-nx-2)/(8*delta^4);

w1=(2*delta^2-1)^2/(16*delta^4);

w2=1/(2^(nx+2)*delta^4);

%%%%Time-update
Sk1k1=utchol(Pk1k1);

Xk1k1_1=repmat(xk1k1,1,nPts1)+Sk1k1*CPtArray1;

Xk1k1_2=repmat(xk1k1,1,nPts2)+Sk1k1*CPtArray2;

Xk1k1=[xk1k1 Xk1k1_1 Xk1k1_2];

Xkk1=ckf_ProssEq(Xk1k1);  

xkk1=w0*Xkk1(:,1)+ w1*sum(Xkk1(:,2:nPts1+1),2)+w2*sum(Xkk1(:,nPts1+2:end),2);

Pkk1=w0*Xkk1(:,1)*Xkk1(:,1)'+w1*Xkk1(:,2:nPts1+1)*Xkk1(:,2:nPts1+1)'+w2*Xkk1(:,nPts1+2:end)*Xkk1(:,nPts1+2:end)'-xkk1*xkk1'+Q;    

%%%%Measurement-update
Skk1=utchol(Pkk1);

Xi_1=repmat(xkk1,1,nPts1)+Skk1*CPtArray1;

Xi_2=repmat(xkk1,1,nPts2)+Skk1*CPtArray2;

Xi=[xkk1 Xi_1 Xi_2];

Zi=ckf_Mst(Xi,t);

zkk1=w0*Zi(:,1)+w1*sum(Zi(:,2:nPts1+1),2)+w2*sum(Zi(:,nPts1+2:end),2);

Pzzkk1=w0*Zi(:,1)*Zi(:,1)'+w1*Zi(:,2:nPts1+1)*Zi(:,2:nPts1+1)'+w2*Zi(:,nPts1+2:end)*Zi(:,nPts1+2:end)'-zkk1*zkk1'+R;  

Pxzkk1=w0*Xi(:,1)*Zi(:,1)'+w1*Xi(:,2:nPts1+1)*Zi(:,2:nPts1+1)'+w2*Xi(:,nPts1+2:end)*Zi(:,nPts1+2:end)'-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);   

xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';  
