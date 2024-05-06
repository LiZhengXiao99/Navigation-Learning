function [xkk,Skk] = sthckf(xkk,Skk,z,Q,R,v1,v2,v3)

%%%%%%Transform covariance matrix into scale matrix
Skk=(v3-2)/v3*Skk;

%%%%%%%%%%
nx=size(xkk,1);        

nz=size(z,1);

nPts=2*nx^2+1; 

w1=2*(nx+v3-2)/((nx+2)*(v3-2));

w2=(v3-4)/((nx+2)^2*(v3-2));

w3=(v3-4)*(4-nx)/(2*(nx+2)^2*(v3-2));

w=[w1 repmat(w2,1,2*nx*(nx-1)) repmat(w3,1,2*nx)];

W=diag(w);

%%%%%%%%%%Time update
Xk1k1=STHCR(xkk,Skk,v3);        

Xkk1=ckf_ProssEq(Xk1k1);         

xkk1=Xkk1*W*ones(nPts,1);

Pkk1=(v3-2)/v3*(Xkk1*W*Xkk1'-xkk1*xkk1') + ((v3-2)/v3)*(v1/(v1-2))*Q;     

%%%%%%%%%%Measurement update

Xi=STHCR(xkk1,Pkk1,v3);   

Zi=ckf_Mst(Xi);
    
zkk1=Zi*W*ones(nPts,1);

Pzzkk1=(v3-2)/v3*(Zi*W*Zi'-zkk1*zkk1') + ((v3-2)/v3)*(v2/(v2-2))*R;  

Pxzkk1=(v3-2)/v3*(Xi*W*Zi'-xkk1*zkk1');   

%%%%%%%
Wk=Pxzkk1*inv(Pzzkk1);        

xkk=xkk1+Wk*(z-zkk1);

deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);

Skk=(v3+deta2)/(v3+nz)*(Pkk1-Wk*Pzzkk1*Wk');

vvv=v3+nz;

%%%%%%%Transform scale matrix into covariance matrix
Skk=vvv/(vvv-2)*Skk;

