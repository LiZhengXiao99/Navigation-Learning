function [xkk,Pkk]=ckf(xk1k1,Pk1k1,z,Q,R,t)

%%%%Preparation
nx=size(xk1k1,1);                                         

nPts=2*nx;      

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%Time-update
Sk1k1=utchol(Pk1k1);

Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;       
          
Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;             

%%%%Measurement-update
Skk1=utchol(Pkk1);

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;
    
Zi=ckf_Mst(Xi,t);
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
    
xkk=xkk1+Wk*(z-zkk1);

Pkk=Pkk1-Wk*Pzzkk1*Wk';   
