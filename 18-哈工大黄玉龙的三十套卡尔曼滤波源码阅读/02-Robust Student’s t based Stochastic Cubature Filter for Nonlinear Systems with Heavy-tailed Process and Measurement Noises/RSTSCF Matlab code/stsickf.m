function [xkk,Skk] = stsickf(xkk,Skk,z,Q,R,v1,v2,v3,Nmax,xp,yp)

%%%%%%%%
Skk=(v3-2)/v3*Skk;

%%%%%%%%
xk1k1 = xkk;

nx=size(xkk,1);        

nz=size(z,1);

nPts=2*nx;

CPtArray=[eye(nx) -eye(nx)];

%%%%%%%%%Time update
%%%%%%%%%Initial value for integral
xkk1=zeros(nx,1);

Pkk1=zeros(nx,nx);

for N=1:Nmax
    
    %%%%%%%%Produce the random orthogonal matrix with Haar distribution
    X=randn(nx);
    
    [B,rr]=qr(X);
    
    %%%%%%%%Produce the random postion with beta distribution
    tao=betarnd((nx+2)/2,(v3-2)/2);
    
    rou=sqrt(tao/(1-tao));
    
    %%%%%%%%Calculate weights
    w0=1-nx/((v3-2)*rou^2);
    
    w1=0.5/((v3-2)*rou^2);
    
    %%%%%%%%Produce cubature points
    Skk1=utchol(v3*Skk)*rou*B;  
    
    Xk1k1=[xk1k1 (repmat(xk1k1,1,nPts)+Skk1*CPtArray)];
    
    %%%%%%%%Propagate cubature points
    Xkk1=ckf_ProssEq(Xk1k1);                 
    
    %%%%%%%%Calculate integral
    SR1=w0*Xkk1(:,1)+w1*sum(Xkk1(:,2:2*nx+1),2);
    
    xkk1=xkk1+SR1/Nmax;
    
    SR2=w0*Xkk1(:,1)*Xkk1(:,1)'+w1*Xkk1(:,2:2*nx+1)*Xkk1(:,2:2*nx+1)';
    
    Pkk1=Pkk1+SR2/Nmax;

end

Pkk1=(v3-2)/v3*(Pkk1-xkk1*xkk1')+((v3-2)/v3)*(v1/(v1-2))*Q; 

%%%%%%%%Measurement update
%%%%%%%%%Initial value for integral
zkk1=zeros(nz,1);

Pzzkk1=zeros(nz,nz);

Pxzkk1=zeros(nx,nz);

for N=1:Nmax
    
    %%%%%%%%Produce the random orthogonal matrix with Haar distribution
    X=randn(nx);
    
    [B,rr]=qr(X);
    
    %%%%%%%%Produce the random postion with beta distribution
    tao=betarnd((nx+2)/2,(v3-2)/2);
    
    rou=sqrt(tao/(1-tao));
    
    %%%%%%%%Calculate weights
    w0=1-nx/((v3-2)*rou^2);
    
    w1=0.5/((v3-2)*rou^2);
    
    %%%%%%%%Produce cubature points
    Skk1=utchol(v3*Pkk1)*rou*B;  
    
    Xi =[xkk1 (repmat(xkk1,1,nPts)+Skk1*CPtArray)];
    
    %%%%%%%%Propagate cubature points
    Zi=ckf_Mst(Xi,xp,yp);
    
    %%%%%%%%Calculate integral
    SR1=w0*Zi(:,1)+w1*sum(Zi(:,2:2*nx+1),2);
    
    zkk1=zkk1+SR1/Nmax;
    
    SR2=w0*Zi(:,1)*Zi(:,1)'+w1*Zi(:,2:2*nx+1)*Zi(:,2:2*nx+1)';

    Pzzkk1=Pzzkk1+SR2/Nmax;

    SR3=w0*Xi(:,1)*Zi(:,1)'+w1*Xi(:,2:2*nx+1)*Zi(:,2:2*nx+1)';

    Pxzkk1=Pxzkk1+SR3/Nmax;
    
end

Pzzkk1=(v3-2)/v3*(Pzzkk1-zkk1*zkk1')+((v3-2)/v3)*(v2/(v2-2))*R;

Pxzkk1=(v3-2)/v3*(Pxzkk1-xkk1*zkk1');

%%%%%%%
Wk=Pxzkk1*inv(Pzzkk1);        

xkk=xkk1+Wk*(z-zkk1);

deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);

Skk=(v3+deta2)/(v3+nz)*(Pkk1-Wk*Pzzkk1*Wk');

vvv=v3+nz;

%%%%%%%
Skk=vvv/(vvv-2)*Skk;

