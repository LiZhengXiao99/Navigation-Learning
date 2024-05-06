function [xk1k1,Pk1k1,tk1k1,Tk1k1]=arvbkf(xkk,Pkk,F,H,z,Q,tkk,Tkk,N,v,rou)

%%%%%%Set up
nz=size(z,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
xk1k1=xk1k;

tk1k=rou*tkk;

Tk1k=rou*Tkk;

%%%%%Initialization
E_inv_R=tk1k*inv(Tk1k);

E_lamda=1;

for i=1:N
    
    %%%%%%%%%%
    xk1k1_i=xk1k1;
    
    %%%%%%%Update the distribution of state vector
    D_R=inv(E_inv_R)/E_lamda;

    zk1k=H*xk1k;
    
    Pzzk1k=H*Pk1k*H'+D_R;
    
    Pxzk1k=Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xk1k1=xk1k+Kk*(z-zk1k);
    
    Pk1k1=Pk1k-Kk*H*Pk1k;
    
    %%%%%%%% Determine convergence
    td=norm(xk1k1-xk1k1_i)/norm(xk1k1);
    
    if td<=1e-16
        break;
    end
    
    %%%%%%%Update the distribution of measurement noise covariance matrix
    Ak=(z-H*xk1k1)*(z-H*xk1k1)'+H*Pk1k1*H';
    
    tk1k1=tk1k+1;
    
    Tk1k1=Tk1k+E_lamda*Ak;

    E_inv_R=tk1k1*inv(Tk1k1);
    
    %%%%%%%Update eta and lamda
    alpha=0.5*(v+nz);
    
    beta=0.5*(v+trace(H*Pk1k1*H'*E_inv_R)+trace((z-H*xk1k1)*(z-H*xk1k1)'*E_inv_R));
    
    E_lamda=alpha/beta;
    
end
