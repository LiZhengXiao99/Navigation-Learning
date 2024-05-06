function [xkk,Pkk]=ssmkf_1(xkk,Pkk,F,H,z,Q,R,sigma,v,N,flag)

%%
nz=size(z,1);

nx=size(xkk,1);

%% Time-update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%% Measurement-update
%% Initialization
xkk=xk1k;

Pkk=Pk1k;

lamda1=1;

lamda2=1;

%% Fixed-point iteration
for i=1:N
    
    %% 
    xkk_i=xkk;
        
    %% Update state
    D_Pk1k=Pk1k/lamda1;

    D_R=R/lamda2;

    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %% Check convergence
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=1e-16
        break;
    end
    
    %% Update modified parameters
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama1=trace(Dk1*inv(Pk1k));

    gama2=trace(Dk2*inv(R));
    
    %% SSMKF-exp-F
    if flag==1
        lamda1=exp(0.5*(nx-gama1)/sigma^2);        
        lamda2=exp(0.5*(nz-gama2)/sigma^2);
    end
    
    %% SSMKF-log-F
    if flag==2
        lamda1=(v+nx)/(v+gama1);
        lamda2=(v+nz)/(v+gama2);
    end

    %% SSMKF-sqrt-F
    if flag==3
        lamda1=sqrt((v+nx)/(v+gama1));
        lamda2=sqrt((v+nz)/(v+gama2));
    end

    %% Avoid numerical instability
    if lamda1<1e-8
        lamda1=lamda1+1e-8;
    end
    %% Avoid numerical instability
    if lamda2<1e-8
        lamda2=lamda2+1e-8;
    end
    
end