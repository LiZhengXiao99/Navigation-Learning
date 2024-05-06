function [xkk,Skk]=ckf(xkk,Skk,z,Q,R)

%%%%%%%%%%
nx=size(xkk,1);                                         

nPts=2*nx;    

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%%%%%%%Time update
xk1k1=xkk;

Skk1=utchol(Skk);

Xk1k1=repmat(xk1k1,1,nPts)+Skk1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;       
          
Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;    

%%%%%%%%%%Measurement update
Skk1=utchol(Pkk1);

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;
    
Zi=ckf_Mst(Xi);
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
    
xkk=xkk1+Wk*(z-zkk1);

Skk=Pkk1-Wk*Pzzkk1*Wk';   