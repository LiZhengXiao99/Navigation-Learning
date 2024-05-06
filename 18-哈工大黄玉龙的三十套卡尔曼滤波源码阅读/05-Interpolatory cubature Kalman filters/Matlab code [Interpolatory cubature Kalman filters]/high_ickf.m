function [xkk,Pkk]=high_ickf(xk1k1,Pk1k1,z,Q,R,lamda_1,lamda_2)

%%%%Preparation
nx=size(xk1k1,1);   

nPts1=2*nx;  

nPts2=2*nx*(nx-1);  

nPts3=2*nx;  

CPtArray1=lamda_1*[eye(nx) -eye(nx)];

CPtArray2=interpolatory_sigma_point(nx,lamda_1); 

CPtArray3=lamda_2*[eye(nx) -eye(nx)];

w0=1-nx/lamda_1^2+nx*(nx-1)/(2*lamda_1^4)+nx*(3-lamda_1^2)/(lamda_1^2*lamda_2^2);

w1=0.5*(1/lamda_1^2+(3-lamda_1^2)/(lamda_1^2*(lamda_1^2-lamda_2^2))-(nx-1)/lamda_1^4);

w2=1/(4*lamda_1^4);

w3=(3-lamda_1^2)/(2*lamda_2^2*(lamda_2^2-lamda_1^2));

%%%%Time-update
Sk1k1=utchol(Pk1k1);
    
Xk1k1_1=repmat(xk1k1,1,nPts1)+Sk1k1*CPtArray1;
    
Xk1k1_2=repmat(xk1k1,1,nPts2)+Sk1k1*CPtArray2;
    
Xk1k1_3=repmat(xk1k1,1,nPts3)+Sk1k1*CPtArray3;
    
Xk1k1=[xk1k1 Xk1k1_1 Xk1k1_2 Xk1k1_3];
    
Xkk1=ckf_ProssEq(Xk1k1);  
    
xkk1=w0*Xkk1(:,1)+w1*sum(Xkk1(:,2:nPts1+1),2)+w2*sum(Xkk1(:,nPts1+2:nPts1+nPts2+1),2)+w3*sum(Xkk1(:,nPts1+nPts2+2:end),2);
    
Pkk1=w0*Xkk1(:,1)*Xkk1(:,1)'+w1*Xkk1(:,2:nPts1+1)*Xkk1(:,2:nPts1+1)'+w2*Xkk1(:,nPts1+2:nPts1+nPts2+1)*Xkk1(:,nPts1+2:nPts1+nPts2+1)'+...
w3*Xkk1(:,nPts1+nPts2+2:end)*Xkk1(:,nPts1+nPts2+2:end)'-xkk1*xkk1'+Q;  

%%%%Measurement-update
Skk1=utchol(Pkk1);

Xi_1=repmat(xkk1,1,nPts1)+Skk1*CPtArray1;

Xi_2=repmat(xkk1,1,nPts2)+Skk1*CPtArray2;

Xi_3=repmat(xkk1,1,nPts3)+Skk1*CPtArray3;

Xi=[xkk1 Xi_1 Xi_2 Xi_3];

Zi=ckf_Mst(Xi);

zkk1=w0*Zi(:,1)+w1*sum(Zi(:,2:nPts1+1),2)+w2*sum(Zi(:,nPts1+2:nPts1+nPts2+1),2)+w3*sum(Zi(:,nPts1+nPts2+2:end),2);

Pzzkk1=w0*Zi(:,1)*Zi(:,1)'+w1*Zi(:,2:nPts1+1)*Zi(:,2:nPts1+1)'+w2*Zi(:,nPts1+2:nPts1+nPts2+1)*Zi(:,nPts1+2:nPts1+nPts2+1)'+...
w3*Zi(:,nPts1+nPts2+2:end)*Zi(:,nPts1+nPts2+2:end)'-zkk1*zkk1'+R;  

Pxzkk1=w0*Xi(:,1)*Zi(:,1)'+w1*Xi(:,2:nPts1+1)*Zi(:,2:nPts1+1)'+w2*Xi(:,nPts1+2:nPts1+nPts2+1)*Zi(:,nPts1+2:nPts1+nPts2+1)'+...
w3*Xi(:,nPts1+nPts2+2:end)*Zi(:,nPts1+nPts2+2:end)'-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);   

xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';  
