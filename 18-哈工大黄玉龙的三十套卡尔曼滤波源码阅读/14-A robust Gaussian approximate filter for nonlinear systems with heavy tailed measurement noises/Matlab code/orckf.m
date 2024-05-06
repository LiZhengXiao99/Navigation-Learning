function [xkk,Skk]=orckf(xkk,Skk,z,Q,R,v,N)

%%%%%%Time update
nx=size(xkk,1);    

nz=size(z,1);

nPts=2*nx;         

Xk1k1=CR(xkk,Skk);

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;       

Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;             

%%%%%%Measurement update
lamda=1;

for i=1:N
    
    %%%%%%Update state
    Xi=CR(xkk1,Pkk1);
    
    Zi=ckf_Mst(Xi);
    
    zkk1=sum(Zi,2)/nPts;  
    
    Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R/lamda;   
    
    Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

    Wk=Pxzkk1*inv(Pzzkk1);      

    xkk=xkk1+Wk*(z-zkk1);
    
    Skk=Pkk1-Wk*Pzzkk1*Wk';   
    
    %%%%%%Update lamda
    Xkk=CR(xkk,Skk);
    
    Zkk=ckf_Mst(Xkk);
    
    Zkk=repmat(z,1,nPts)-Zkk;
    
    Dk=Zkk*Zkk'/nPts;
    
    gama=trace(Dk*inv(R));
         
    lamda=(v+nz)/(v+gama);
    
end

