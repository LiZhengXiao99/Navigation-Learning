function [xk1k1,Pk1k1,tk1k,Pttk1k,K,K1]=correlated_ckf(xkk,Pkk,z,Q,R,S)

%%%%%%%%%%%%%%%%%%%Preparation
Pseudo_R=R-S'*inv(Q)*S;

nx=size(xkk,1);          

nPts=2*nx;      

CPtArray=sqrt(nPts/2)*[eye(nPts/2) -eye(nPts/2)];

nPts_1=4*nx;       

CPtArray_1=sqrt(nPts_1/2)*[eye(nPts_1/2) -eye(nPts_1/2)];

%%%%%%%%%%%%%%%%%%%Time update
Skk=utchol(Pkk);

Xkk=repmat(xkk,1,nPts)+Skk*CPtArray;

Xk1k=ckf_ProssEq(Xkk);        

xk1k=sum(Xk1k,2)/nPts;       

Pk1k=Xk1k*Xk1k'/nPts-xk1k*xk1k'+Q;        

%%%%%%%%%%%%%%%%%%%Measurement update

Pk1kk=Xk1k*Xkk'/nPts-xk1k*xkk';

ck1k=[xk1k;xkk];

Pcck1k=[Pk1k Pk1kk;Pk1kk' Pkk];

Scck1k=utchol(Pcck1k);

Ck1k=repmat(ck1k,1,nPts_1)+Scck1k*CPtArray_1;

Xi_k1=Ck1k(1:nx,:);

Xi_k=Ck1k(nx+1:2*nx,:);

Zi=Pseudo_ckf_Mst(Xi_k1,Xi_k,S,Q);

zk1k=sum(Zi,2)/nPts_1;         

Pzzk1k=Zi*Zi'/nPts_1-zk1k*zk1k'+Pseudo_R;

Pxzk1k=Xi_k1*Zi'/nPts_1-xk1k*zk1k';

Wk=Pxzk1k*inv(Pzzk1k);        

xk1k1 = xk1k + Wk*(z - zk1k);  
    
Pk1k1=Pk1k-Wk*Pzzk1k*Wk';   

%%%%%%%%%%%%%%%%%%%%%%Auxiliary parameters for smoothing
tk1k=[xk1k;zk1k];

Pttk1k=[Pk1k Pxzk1k;Pxzk1k' Pzzk1k];

Pxzkk1k=Xi_k*Zi'/nPts_1-xkk*zk1k';

Pxtkk1k=[Pk1kk' Pxzkk1k];

K=Pxtkk1k*inv(Pttk1k);                         %%nx*(nx+nz)

K1=Pk1kk'*inv(Pk1k1)-Pxzkk1k*Wk'*inv(Pk1k1);   %%%nx*nx





