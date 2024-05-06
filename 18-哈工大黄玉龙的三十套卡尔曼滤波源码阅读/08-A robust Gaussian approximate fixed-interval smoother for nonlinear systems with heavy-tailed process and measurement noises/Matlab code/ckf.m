function [xkk,Skk,xkk1,Pkk1,Pkk1k]=ckf(xkk,Skk,F,z,Q,R)

%%%%%%%%%
nx=size(xkk,1);    

nPts=2*nx;  

%%%%%%%%%Time update
xk1k1=xkk;

Sk1k1=Skk;

xkk1=F*xk1k1;

Pkk1=F*Sk1k1*F'+Q;

Pkk1=(Pkk1+Pkk1')/2;        %%%%%Keep symmetry

Pkk1k=Sk1k1*F';

%%%%%%%%%Measurement update
Xi=CR(xkk1,Pkk1);

Zi=ckf_Mst(Xi);
    
zkk1=sum(Zi,2)/nPts;           

Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;

Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

Wk=Pxzkk1*inv(Pzzkk1);            
  
xkk=xkk1+Wk*(z-zkk1);  

Skk=Pkk1-Wk*Pzzkk1*Wk';   


   