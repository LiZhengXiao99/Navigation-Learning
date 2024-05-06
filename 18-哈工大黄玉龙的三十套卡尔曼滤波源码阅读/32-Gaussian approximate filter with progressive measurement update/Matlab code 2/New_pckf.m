function [xkk,Skk]=New_pckf(xkk,Skk,z,Q,R,N)

%%%%%%%%
nx=size(xkk,1);                                         

nPts=2*nx;         

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%%%%%Time update
Sk1k1=utchol(Skk);

xk1k1=xkk;

Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;       

Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;             

%%%%%%%%Measurement update
for i=1:N
    
    Skk1=utchol(Pkk1);
    
    Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;
    
    Zi=ckf_Mst(Xi);
    
    zkk1=sum(Zi,2)/nPts;      
    
    Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+N*R;

    Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';
    
    Kk=Pxzkk1*inv(Pzzkk1); 
    
    xkk=xkk1+Kk*(z - zkk1);
    
    Pkk=Pkk1-Kk*Pzzkk1*Kk'; 

    %%%%%%%%Initial value fore next iteration
    xkk1=xkk;
    
    Pkk1=Pkk;
    
end

xkk=xkk1;

Skk=Pkk1;
