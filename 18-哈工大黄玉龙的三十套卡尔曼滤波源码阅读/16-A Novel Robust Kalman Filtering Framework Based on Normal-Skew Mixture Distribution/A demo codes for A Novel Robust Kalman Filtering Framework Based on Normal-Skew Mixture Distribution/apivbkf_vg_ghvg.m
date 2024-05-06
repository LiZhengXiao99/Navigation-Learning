function [xkk,Pkk]=apivbkf_vg_ghvg(xkk,Pkk,F,H,z,Q,R,f0,u0,e0,g0,v1,E_v,beta2_bar,sigama2,N)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
xkk=xk1k;

Pkk=Pk1k;

E_Pk1k=Pk1k;

E_i_Pk1k=inv(E_Pk1k);

E_R=R;

E_i_R=inv(E_R);

F0=f0*Pk1k;

U0=u0*R;

%%%%%Initial parameters
E_kasai=1;

E_k1_kasai=E_kasai;

E_log_kasai=0;

E_t=4/5;

E_log_tao=psi(g0)-psi(1);

E_log_1_tao=psi(1-g0)-psi(1);

%%%%%Initial parameters 
E_lamda=1;

E_k2_lamda=1/E_lamda;

E_log_lamda=0;

E_y=4/5;

E_log_pi=psi(e0)-psi(1);

E_log_1_pi=psi(1-e0)-psi(1);

%%%%%Initial parameters 
beta1_bar=zeros(nx,1);
sigama1=0;
E_beta1=beta1_bar;
P_beta1=sigama1*eye(nx);

E_beta2=beta2_bar;
P_beta2=sigama2*eye(nz);

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
    
    %%%%%%%Update state vector
    D_Pk1k=E_Pk1k/(E_t+(1-E_t)*E_k1_kasai); 
    
    D_R=E_R/(E_y+(1-E_y)*E_k2_lamda); 

    qkk=D_Pk1k*[E_t*E_i_Pk1k*xk1k+(1-E_t)*E_k1_kasai*E_i_Pk1k*(xk1k+E_kasai*E_beta1)];
    
    akk=D_R*(1-E_y)*E_k2_lamda*E_i_R*E_lamda*E_beta2;
    
%     zk1k=H*xk1k+akk;
    zk1k=H*qkk+akk;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
%     xkk=xk1k+Kk*(z-zk1k);
    xkk=qkk+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%%Determine convergence
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=1e-16
        break;
    end
    
    %%%%%%%Calculate auxiliary parameters
    Ck=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';

    %%%%%%%Update auxiliary parameter VG-GHSST
    %%%%%eta_1 and eta_3
    eta_1_kasai=(1-E_t)*trace(Ck*E_i_Pk1k);
    eta_1_lamda=(1-E_y)*trace(Dk*E_i_R);
    eta_3_lamda=(1-E_y)*trace((E_beta2*E_beta2'+P_beta2)*E_i_R);
    s_lamd=nz*(1-E_y);

    %%%%%Update kasai (VG)
    kasai=(nx*(1-E_t)-v1-2+sqrt((nx*(1-E_t)-v1-2)^2+4*v1*eta_1_kasai))/(2*eta_1_kasai);
 
    %%%%%Update lamda (GHVG)    
    lamda=(-(s_lamd-E_v+2)+sqrt((s_lamd-E_v+2)^2+4*eta_1_lamda*(eta_3_lamda+E_v)));
    
    %%%%%
    E_kasai=kasai;
    E_k1_kasai=E_kasai;
    E_log_kasai=log(kasai);
    E_log_k1_kasai=E_log_kasai;
    E_kasai2=kasai*kasai;
    %%%%%
    E_lamda=lamda;
    E_k2_lamda=1/lamda;
    E_log_lamda=log(lamda);
    E_log_k2_lamda=-E_log_lamda;
    E_lamda2=lamda*lamda;
    
    %%%%%%%Calculate auxiliary parameters
    Ak=Ck-E_kasai*(xkk-xk1k)*E_beta1'-E_kasai*E_beta1*(xkk-xk1k)'+E_kasai2*(E_beta1*E_beta1'+P_beta1);
    
    Bk=Dk-E_lamda*(z-H*xkk)*E_beta2'-E_lamda*E_beta2*(z-H*xkk)'+E_lamda2*(E_beta2*E_beta2'+P_beta2);
    
    %%%%%%%Update shape vector beta1. %%beta1=0 
    sigama1=0;
    
    E_beta1=zeros(nx,1);
    
    P_beta1=sigama1*eye(nx);
    
    %%%%%%%Update shape vector beta2.
    Wk2=E_R/((1-E_y)*E_k2_lamda);
    
    Kbeta2=sigama2*E_lamda*inv(E_lamda*E_lamda*sigama2*eye(nz)+Wk2);
    
    E_beta2=beta2_bar+Kbeta2*(z-H*xkk-E_lamda*beta2_bar);
    
    P_beta2=(eye(nz)-Kbeta2*E_lamda)*sigama2;
    
    %%%%%%%Update scale matrix Pk1k
    fkk=f0+1; 
    
    Fkk=E_t*Ck+(1-E_t)*E_k1_kasai*Ak+F0;
    
    E_i_Pk1k=fkk*inv(Fkk);
    
    E_Pk1k=inv(E_i_Pk1k);
    
    %%%%%%%Update scale matrix Rk
    ukk=u0+1;
    
    Ukk=E_y*Dk+(1-E_y)*E_k2_lamda*Bk+U0;
    
    E_i_R=ukk*inv(Ukk);
    
    E_R=inv(E_i_R);
    
    %%%%%%%Update mixing parameter t
    pt1=exp(E_log_tao-0.5*trace(Ck*E_i_Pk1k));
    
    pt0=exp(E_log_1_tao+0.5*nx*E_log_k1_kasai-0.5*E_k1_kasai*trace(Ak*E_i_Pk1k));
    
    if pt1<=1e-98
        pt1=pt1+1e-98;
    end
    
    if pt0<=1e-98
        pt0=pt0+1e-98;
    end
    
    E_t=pt1/(pt1+pt0);

    %%%%%%%Update mixing parameter y
    py1=exp(E_log_pi-0.5*trace(Dk*E_i_R));
    
    py0=exp(E_log_1_pi+0.5*nz*E_log_k2_lamda-0.5*E_k2_lamda*trace(Bk*E_i_R));
    
    if py1<=1e-98  
        py1=py1+1e-98;
    end
    
    if py0<=1e-98
        py0=py0+1e-98;
    end
    
    E_y=py1/(py1+py0);
    
    if E_y>0.999999  %%%%%%%%%%%%%%%%%%%
        E_y=0.999999;
    end

    %%%%%%%Update \tau
    gk=g0+E_t;
    
    mk=2-E_t-g0;
    
    E_log_tao=psi(gk)-psi(gk+mk);
    
    E_log_1_tao=psi(mk)-psi(gk+mk);
    
    %%%%%%%Update \pi
    ek=e0+E_y;
    
    nk=2-E_y-e0;
    
    E_log_pi=psi(ek)-psi(ek+nk);
    
    E_log_1_pi=psi(nk)-psi(ek+nk);

end


