function [xkk,Skk]=scif(xkk,Skk,z,SQ,SR,Xs,Ys,T0)

%%%%
nx=size(xkk,1);   

nz=size(z,1);

ns=size(Xs,2);

nPts=2*nx;  

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%Time update

xk1k1=xkk;

Sk1k1=Skk; 

Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1,T0);      

xkk1=sum(Xkk1,2)/nPts; 

Xxkk1=sqrt(1/nPts)*(Xkk1-repmat(xkk1,1,nPts));    

Skk1=Sroot([Xxkk1 SQ]);

S_Ykk1=inv(Skk1');

ykk1=S_Ykk1*S_Ykk1'*xkk1;

%%%%Measurement update

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;

Xxi= sqrt(1/nPts)*(Xi-repmat(xkk1,1,nPts));

Zi =ckf_Mst(Xi,Xs,Ys);    %%%%nz*(2nx*ns)

ykk=ykk1;

S_Ik=S_Ykk1;

for s=1:ns
    
    Zi_s=Zi(:,nPts*(s-1)+1:nPts*s);
    
    zkk1_s = sum(Zi_s,2)/nPts;
    
    Zzi_s=sqrt(1/nPts)*(Zi_s-repmat(zkk1_s,1,nPts));
    
    T=Sroot([Zzi_s SR;Xxi zeros(nx,nz)]);
    
    T11=T(1:nz,1:nz);
    
    T21=T(nz+1:nz+nx,1:nz);
    
    R_s=SR*SR';
    
    S_R=utchol(inv(R_s));
    
    S_Ik_s=S_Ykk1*S_Ykk1'*T21*T11'*S_R;
    
    ik_s = S_Ik_s*S_R'*(z(:,s)-zkk1_s)+S_Ik_s*S_Ik_s'*xkk1;
    
    ykk=ykk+ik_s;
    
    S_Ik = [S_Ik S_Ik_s];
    
end

S_Ykk=Sroot(S_Ik);

Skk=inv(S_Ykk');

xkk=Skk*Skk'*ykk;




    
    


