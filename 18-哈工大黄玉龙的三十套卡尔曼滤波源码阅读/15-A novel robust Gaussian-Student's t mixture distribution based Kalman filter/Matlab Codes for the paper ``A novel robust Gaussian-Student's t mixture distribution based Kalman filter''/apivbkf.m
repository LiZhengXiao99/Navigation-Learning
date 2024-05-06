function [xkk,Pkk]=apivbkf(xkk,Pkk,F,H,z,Q,R,N,e0,f0,g0,h0,v1,v2)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
xkk=xk1k;

Pkk=Pk1k;

%%%%%Initial parameters 
E_lamda=1;

E_log_lamda=0;

E_y=1;

E_log_pi=psi(e0)-psi(e0+f0);

E_log_1_pi=psi(f0)-psi(e0+f0);

%%%%%Initial parameters
E_kasai=1;

E_log_kasai=0;

E_t=1;

E_log_tao=psi(g0)-psi(g0+h0);

E_log_1_tao=psi(h0)-psi(g0+h0);

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
    
    %%%%%%%Update state vector
    D_Pk1k=Pk1k/(E_t+(1-E_t)*E_kasai);
    
    D_R=R/(E_y+(1-E_y)*E_lamda);
    
    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%%Determine convergence
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=1e-16
        break;
    end
    
    %%%%%%%Calculate auxiliary parameters
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;

    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama1=trace(Dk1*inv(Pk1k));

    gama2=trace(Dk2*inv(R));

    %%%%%%%Update mixing parameter y
    py1=exp(E_log_pi-0.5*gama2);
    
    py0=exp(E_log_1_pi+0.5*nz*E_log_lamda-0.5*E_lamda*gama2);
    
    if py1<=1e-98
        py1=py1+1e-98;
    end
    
    if py0<=1e-98
        py0=py0+1e-98;
    end
    
    E_y=py1/(py1+py0);
    
    %%%%%%%Update mixing parameter t
    pt1=exp(E_log_tao-0.5*gama1);
    
    pt0=exp(E_log_1_tao+0.5*nx*E_log_kasai-0.5*E_kasai*gama1);
    
    if pt1<=1e-98
        pt1=pt1+1e-98;
    end
    
    if pt0<=1e-98
        pt0=pt0+1e-98;
    end
    
    E_t=pt1/(pt1+pt0);

    %%%%%%%Update auxiliary parameter \lambda
    alfa_k=0.5*nz*(1-E_y)+0.5*v2;
    
    beta_k=0.5*gama2*(1-E_y)+0.5*v2;
    
    E_lamda=alfa_k/beta_k;
    
    E_log_lamda=psi(alfa_k)-log(beta_k);
    
    %%%%%%%Update auxiliary parameter \xi
    eta_k=0.5*nx*(1-E_t)+0.5*v1;
    
    theta_k=0.5*gama1*(1-E_t)+0.5*v1;
    
    E_kasai=eta_k/theta_k;
    
    E_log_kasai=psi(eta_k)-log(theta_k);
    
    %%%%%%%Update \pi
    ek=e0+E_y;
    
    fk=f0+1-E_y;
    
    E_log_pi=psi(ek)-psi(ek+fk);
    
    E_log_1_pi=psi(fk)-psi(ek+fk);
    
    %%%%%%%Update \tau
    gk=g0+E_t;
    
    hk=h0+1-E_t;
    
    E_log_tao=psi(gk)-psi(gk+hk);
    
    E_log_1_tao=psi(hk)-psi(gk+hk);

end
