function [xkk,Pkk]=high_ukf(xk1k1,Pk1k1,z,Q,R,t,kai)

%%%%Preparation
k2=kai;

nz=size(z,1);

nx=size(xk1k1,1);

nPts=2*nx^2+1;

%%%%Weight matrix set
w=zeros(1,nPts);     

w0=(-2*nx^2+(4-2*nx)*k2^2+(4*k2+4)*nx)/((4-nx)*(nx+k2)^2);

w1=(k2+2-nx)^2/(2*(4-nx)*(nx+k2)^2);

w2=1/(nx+k2)^2;

w(1)=w0;

w(2:(2*nx*(nx-1)+1))=repmat(w2,1,2*nx*(nx-1));

w((2*nx*(nx-1)+2):(2*nx^2+1))=repmat(w1,1,2*nx);

%%%%Sigma points
CPtArray=high_sigma_point(nx,k2);

%%%%Time-update
Sk1k1=utchol(Pk1k1);
    
Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);

xkk1=zeros(nx,1);

Pkk1=zeros(nx,nx);

for j=1:nPts
    
    xkk1=xkk1+w(j)*Xkk1(:,j);
    
    Pkk1=Pkk1+w(j)*Xkk1(:,j)*Xkk1(:,j)';
    
end

Pkk1=Pkk1-xkk1*xkk1'+Q;  

%%%%Measurement-update
Skk1=utchol(Pkk1);

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;

Zi=ckf_Mst(Xi,t);

zkk1=zeros(nz,1);

Pzzkk1=zeros(nz,nz);

Pxzkk1=zeros(nx,nz);

for j=1:nPts
    
    zkk1=zkk1+w(j)*Zi(:,j);        
    
    Pzzkk1=Pzzkk1+w(j)*Zi(:,j)*Zi(:,j)';
    
    Pxzkk1=Pxzkk1+w(j)*Xi(:,j)*Zi(:,j)';
    
end

Pzzkk1=Pzzkk1-zkk1*zkk1'+R;

Pxzkk1=Pxzkk1-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);           
    
xkk=xkk1+Wk*(z-zkk1);  

Pkk=Pkk1-Wk*Pzzkk1*Wk';
