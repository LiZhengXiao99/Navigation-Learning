function [xkk,Skk,xkk1,Pkk1,Pkk1k]=ckf(xkk,Skk,t,z,a,b,d,q,Q,r,R)

nx=size(xkk,1);    

nPts=2*nx;  

%%%%%%%%%Time update
xk1k1=xkk;

Sk1k1=Skk;

Xk1k1=CR(xk1k1,Sk1k1);

Xkk1=ckf_ProssEq(a,b,Xk1k1,t)+q;         

xkk1=sum(Xkk1,2)/nPts;              

Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;            
                          
Pkk1=(Pkk1+Pkk1')/2;        

Pkk1k=Xk1k1*Xkk1'/nPts-xk1k1*xkk1';

%%%%%%%%%Measurement update
Xi=CR(xkk1,Pkk1);

Zi=ckf_Mst(d,Xi)+r;
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
  
xkk=xkk1+Wk*(z-zkk1);  

Skk=Pkk1-Wk*Pzzkk1*Wk';   


   