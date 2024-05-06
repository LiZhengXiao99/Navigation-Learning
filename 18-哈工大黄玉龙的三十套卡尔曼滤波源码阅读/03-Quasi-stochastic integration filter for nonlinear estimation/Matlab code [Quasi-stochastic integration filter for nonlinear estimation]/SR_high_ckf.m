function [xkk,Pkk]=SR_high_ckf(xk1k1,Pk1k1,z,Q,R,t,Nmax)

%%%%Preparation
k2=2;%%%%High-degree CKF

nx=size(xk1k1,1);      

nz=size(z,1);

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
%%%%Initial values for integrals
xkk1=zeros(nx,1);

Pkk1=zeros(nx,nx);

for N=1:Nmax
    
    X=randn(nx);
    
    [B,rr]=qr(X);
    
    Sk1k1=utchol(Pk1k1)*B;
    
    Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

    Xkk1=ckf_ProssEq(Xk1k1);         
    
    SR1=zeros(nx,1);
    
    for j=1:nPts
        
        SR1=SR1+w(j)*Xkk1(:,j);

    end
    
    xkk1=xkk1+(SR1-xkk1)/N;
    
    SR2=zeros(nx);
    
    for j=1:nPts
        
        SR2=SR2+w(j)*(Xkk1(:,j)-xkk1)*(Xkk1(:,j)-xkk1)';
        
    end
    
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
    
    SR1=zeros(nz,1);
    
    for j=1:nPts
        
        SR1=SR1+w(j)*Zi(:,j);

    end
    
    zkk1=zkk1+(SR1-zkk1)/N;
    
    SR2=zeros(nz);
    
    for j=1:nPts
        
        SR2=SR2+w(j)*(Zi(:,j)-zkk1)*(Zi(:,j)-zkk1)';
        
    end
    
    Pzzkk1=Pzzkk1+(SR2-Pzzkk1)/N;
    
    SR3=zeros(nx,nz);
    
    for j=1:nPts
        
        SR3=SR3+w(j)*(Xi(:,j)-xkk1)*(Zi(:,j)-zkk1)';
        
    end
    
    Pxzkk1=Pxzkk1+(SR3-Pxzkk1)/N;
    
end

Pzzkk1=Pzzkk1+R;

Wk=Pxzkk1*inv(Pzzkk1);            

xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';   
