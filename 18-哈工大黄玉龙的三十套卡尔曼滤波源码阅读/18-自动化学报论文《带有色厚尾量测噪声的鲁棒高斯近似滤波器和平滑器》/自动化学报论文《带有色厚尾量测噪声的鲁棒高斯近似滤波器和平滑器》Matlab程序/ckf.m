function [xkk,Skk,xkk1,Pkk1,Pkk1k]=ckf(xk1k1,Sk1k1,z,Q,R)

nx=size(xk1k1,1);    

nPts=2*nx;  

%%%%%%%%%时间更新
Xk1k1=CR(xk1k1,Sk1k1); 

Xkk1=ckf_ProssEq(Xk1k1);                  %%%%计算传播的容积点

xkk1=sum(Xkk1,2)/nPts;

Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;

Pkk1k=Xk1k1*Xkk1'/nPts-xk1k1*xkk1';

%%%%%%%%%量测更新
Xi=CR(xkk1,Pkk1);

Zi=ckf_Mst(Xi);
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
  
xkk=xkk1+Wk*(z-zkk1);  

Skk=Pkk1-Wk*Pzzkk1*Wk';   


   