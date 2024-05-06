function [xk1k1,Pk1k1,uk1k1,Uk1k1,tk1k1,Tk1k1]=ivbkf(xkk,Pkk,F,H,z,Q,ukk,Ukk,tkk,Tkk,N,v,rou)

%%%%%%Set up
nz=size(z,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
xk1k1=xk1k;

Pk1k1=Pk1k;

uk1k=ukk;

Uk1k=Ukk/rou;

tk1k=rou*tkk;

Tk1k=rou*Tkk;

%%%%%Initialization
E_eta=uk1k;

P_eta=Uk1k;

E_lamda=1;

for i=1:N
    
    %%%%%%%Update the distribution of measurement noise covariance matrix
    Ak=(z-H*xk1k1-E_eta)*(z-H*xk1k1-E_eta)'+H*Pk1k1*H'+P_eta;
    
    tk1k1=tk1k+1;
    
    Tk1k1=Tk1k+E_lamda*Ak;

    E_inv_R=tk1k1*inv(Tk1k1);
       
    %%%%%%%%%%
    xk1k1_i=xk1k1;

    %%%%%%%Update the distribution of state vector
    D_R=inv(E_inv_R)/E_lamda;

    zk1k=H*xk1k;
    
    Pzzk1k=H*Pk1k*H'+D_R;
    
    Pxzk1k=Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xk1k1=xk1k+Kk*(z-zk1k-E_eta);
    
    Pk1k1=Pk1k-Kk*H*Pk1k;
    
    %%%%%%%% Determine convergence
    td=norm(xk1k1-xk1k1_i)/norm(xk1k1);
    
    if td<=1e-16
        break;
    end
    
    %%%%%%%Update eta and lamda
    E_R=inv(E_inv_R);
    
    zk1=z-H*xk1k1;
    
    Wk1=Uk1k*inv(Uk1k+E_R);
    
    uk1k1=uk1k+Wk1*(zk1-uk1k);
    
    Uk1k1=Uk1k-Wk1*Uk1k;
 
    alpha=0.5*(v+nz);
    
    beta=0.5*(v+trace(H*Pk1k1*H'*inv(E_R))+trace((zk1-uk1k)*(zk1-uk1k)'*inv(Uk1k+E_R)));
        
    %%%%%%%Compute the expectations
    E_eta=uk1k1;

    P_eta=Uk1k1/E_lamda;
    
    E_lamda=alpha/beta;
    
end
