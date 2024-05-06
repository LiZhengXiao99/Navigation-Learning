function [xkk,Pkk]=new_stkf(xkk,Pkk,F,H,z,Q0,R0,v,N,tau,lambda)

nx=size(xkk,1);

nz=size(z,1);

%%%%%%%%%Time update
xkk1=F*xkk;

Pkk1=F*((v-2)/v*Pkk)*F'+Q0;

Pkk1_0=Pkk1;      

%%%%%%%%%Measurement update
xkk=xkk1;

Pkk=Pkk1;

vvv=v;

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
    
    %%%%%%%Estimate Pkk1 and R
    Ak=(vvv/(vvv-2))*Pkk+(xkk-xkk1)*(xkk-xkk1)';
    
    Bk=(z-H*xkk)*(z-H*xkk)'+(vvv/(vvv-2))*H*Pkk*H';
    
    %%%%%%%
    Pkk1=(1-tau)*Pkk1_0+tau*((vvv-2)/vvv)*Ak;
    
    R=(1-lambda)*R0+lambda*((vvv-2)/vvv)*Bk;
    
    %%%%%%%
    zkk1=H*xkk1;
    
    Pzzkk1=H*Pkk1*H'+R;
    
    Pxzkk1=Pkk1*H';
    
    %%%%%%%Initial update
    Wk=Pxzkk1*inv(Pzzkk1);
    
    xkk=xkk1+Wk*(z-zkk1);
    
    deta2=(z-zkk1)'*inv(Pzzkk1)*(z-zkk1);
    
    Pkk=(v+deta2)/(v+nz)*(Pkk1-Wk*Pzzkk1*Wk');
    
    vvv=v+nz;
    
    %%%%%%%%Judge convergence
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=1e-16
        break;
    end

end

%%%%%%%Final update
Pkk=(v/(v-2))*match_1(xkk,Pkk,vvv,v,100);
