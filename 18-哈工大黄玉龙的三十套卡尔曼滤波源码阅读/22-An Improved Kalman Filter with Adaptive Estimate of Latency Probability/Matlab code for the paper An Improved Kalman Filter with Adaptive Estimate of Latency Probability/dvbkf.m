function [xk1k1,Pk1k1,alfa,beta] = dvbkf(xkk,Pkk,F,H,y,Q,R,alfa0,beta0,rou,N)

nx=size(xkk,1);
nz=size(y,1);

A=zeros(nz,nx);
B=zeros(nz,nz);
C=eye(nz,nz);
Hk=[H A;A H];
Rk=[R B;B R];

%%%%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

Pxxkk1k=Pkk*F';

xak1k=[xk1k;xkk];

Pak1k=[Pk1k Pxxkk1k';Pxxkk1k Pkk];

%%%%%%%%measurement update
yk1k=Hk*xak1k;

Sk=Hk*Pak1k*Hk'+Rk;

Pxyk1k1k=[Pk1k*H' Pxxkk1k'*H'];

Pxykk1k=[Pxxkk1k*H' Pkk*H'];

%Initialization
alfa1=rou*alfa0;
beta1=rou*beta0;
E_gamma=alfa1/(alfa1+beta1);
E_1_gamma=1-E_gamma;
E_log_tau=psi(alfa1)-psi(alfa1+beta1);
E_log_1_tau=psi(beta1)-psi(alfa1+beta1);

yk=[y;y];

%%%%%%%%Variational measurement update
for i=1:N
    
    Rk_1=[R/E_gamma B;B R/E_1_gamma];
    
    Pyyk1k=Sk+Rk_1;
    
    Kks=Pxykk1k*inv(Pyyk1k);
    
    xkk1=xkk+Kks*(yk-yk1k);
    
    Pkk1=Pkk-Kks*Pyyk1k*Kks';

    Kk1=Pxyk1k1k*inv(Pyyk1k);
    
    xk1k1=xk1k+Kk1*(yk-yk1k);
    
    Pk1k1=Pk1k-Kk1*Pyyk1k*Kk1';

    Pxxkk1k1=Pxxkk1k-Kks*Pyyk1k*Kk1';

    xak1k1=[xk1k1;xkk1];
    
    Pak1k1=[Pk1k1 Pxxkk1k1';Pxxkk1k1 Pkk1];
    
    %%%%%%%%Update gamma
    Bk=Hk*Pak1k1*Hk'+(yk-Hk*xak1k1)*(yk-Hk*xak1k1)';
    
    Pr1=exp(E_log_tau-0.5*trace(Bk*[C B;B B]*inv(Rk)))+1e-16;
    Pr2=exp(E_log_1_tau-0.5*trace(Bk*[B B;B C]*inv(Rk)))+1e-16;
    
    %%%%%%%%Compute the expectations
    E_gamma=Pr1/(Pr1+Pr2);
    E_1_gamma=1-E_gamma;

    %%%%%%%%Update tau
    alfa=alfa1+E_gamma;
    beta=beta1+E_1_gamma;
    
    %%%%%%%%Compute the expectations
    E_log_tau=psi(alfa)-psi(alfa+beta);
    E_log_1_tau=psi(beta)-psi(alfa+beta);

end 
