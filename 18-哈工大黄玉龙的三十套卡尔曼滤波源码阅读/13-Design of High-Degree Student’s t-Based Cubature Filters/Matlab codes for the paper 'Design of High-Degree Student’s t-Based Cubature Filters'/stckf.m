function [xkk,Skk] = stckf(xkk,Skk,z,Q,R,v1,v2,v3)

%%%%%%Transform covariance matrix into scale matrix
Skk=(v3-2)/v3*Skk;

%%%%%%%%Time update
nx=size(xkk,1);        

nz=size(z,1);

nPts=2*nx;         

Xk1k1=STCR(xkk,Skk,v3);

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;    

Pkk1=(v3-2)/v3*(Xkk1*Xkk1'/nPts-xkk1*xkk1') + ((v3-2)/v3)*(v1/(v1-2))*Q;         

%%%%%%%%Measurement update
Xi=STCR(xkk1,Pkk1,v3);

Zi=ckf_Mst(Xi);
    
zkk1 = sum(Zi,2)/nPts;     

Pzzkk1=(v3-2)/v3*(Zi*Zi'/nPts-zkk1*zkk1') + ((v3-2)/v3)*(v2/(v2-2))*R;

Pxzkk1=(v3-2)/v3*(Xi*Zi'/nPts-xkk1*zkk1');

%%%%%%%
Wk=Pxzkk1*inv(Pzzkk1);        

xkk=xkk1+Wk*(z-zkk1);

deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);

Skk=(v3+deta2)/(v3+nz)*(Pkk1-Wk*Pzzkk1*Wk');

vvv=v3+nz;

%%%%%%%Transform scale matrix into covariance matrix
Skk=vvv/(vvv-2)*Skk;

