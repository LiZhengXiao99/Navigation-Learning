function [xkk,Pkk]=SR_ckf(xk1k1,Pk1k1,z,Q,R,t,Nmax)

%%%%Preparation
nx=size(xk1k1,1);      

nz=size(z,1);

nPts=2*nx;     

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%Time-update
%%%%Initial values for integrals
xkk1=zeros(nx,1);

Pkk1=zeros(nx,nx);

for N=1:Nmax
    
    X=randn(nx);
    
    [B,rr]=qr(X);
    
    Sk1k1=utchol(Pk1k1)*B;
    
    Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;
    
    Xkk1=ckf_ProssEq(Xk1k1);     
    
    SR1=sum(Xkk1,2)/nPts;   
    
    xkk1=xkk1+(SR1-xkk1)/N;
    
    SR2=(Xkk1-repmat(xkk1,1,nPts))*(Xkk1-repmat(xkk1,1,nPts))'/nPts;
    
    Pkk1=Pkk1+(SR2-Pkk1)/N;
    
end

Pkk1=Pkk1+Q;

%%%%Measurement-update
%%%%Initial values for integrals
zkk1=zeros(nz,1);

Pzzkk1=zeros(nz,nz);

Pxzkk1=zeros(nx,nz);

for N=1:Nmax
    
     X=randn(nx);
     
    [B,rr]=qr(X);
    
    Skk1=utchol(Pkk1)*B;
    
    Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;
    
    Zi=ckf_Mst(Xi,t);
    
    SR1=sum(Zi,2)/nPts; 
    
    zkk1=zkk1+(SR1-zkk1)/N;
    
    SR2=(Zi-repmat(zkk1,1,nPts))*(Zi-repmat(zkk1,1,nPts))'/nPts;
    
    Pzzkk1=Pzzkk1+(SR2-Pzzkk1)/N;
    
    SR3=(Xi-repmat(xkk1,1,nPts))*(Zi-repmat(zkk1,1,nPts))'/nPts;
    
    Pxzkk1=Pxzkk1+(SR3-Pxzkk1)/N;
end
 
Pzzkk1=Pzzkk1+R;

Wk=Pxzkk1*inv(Pzzkk1);            

xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';   


