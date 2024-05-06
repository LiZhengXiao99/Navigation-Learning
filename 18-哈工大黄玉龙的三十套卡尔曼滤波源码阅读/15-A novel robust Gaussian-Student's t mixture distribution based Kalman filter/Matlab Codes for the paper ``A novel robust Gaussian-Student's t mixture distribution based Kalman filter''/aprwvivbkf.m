function [xkk,Pkk]=aprwvivbkf(xkk,Pkk,F,H,z,Q,R,tao_p,tao_r,a,b,c,d,N)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
%%%%%Initial values for variational iterations
xkk=xk1k;

Pkk=Pk1k;

u0=(nx+1+tao_p);

U0=tao_p*Pk1k;

t0=(nz+1+tao_r);

T0=tao_r*R;

E_i_Pk1k=(u0-nx-1)*inv(U0);

E_i_R=(t0-nz-1)*inv(T0);

E_kasai=1;

E_lamda=1;

E_w=a/b;

E_v=c/d;

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
    
    %%%%%Update state vector
    %%%%%Calculate modified Pk1k and modified R
    D_Pk1k=inv(E_i_Pk1k)/E_kasai;
    
    D_Pk1k=(D_Pk1k+D_Pk1k')/2;
    
    D_R=inv(E_i_R)/E_lamda;
    
    %%%%%Measurement update
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
    
    %%%%%%%Update auxiliary parameter \xi
    Ak=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    gama1=trace(Ak*E_i_Pk1k);
    
    eta_kk=0.5*(E_w+nx);
    
    theta_kk=0.5*(E_w+gama1);
    
    E_kasai=eta_kk/theta_kk;
    
    %%%%%%%Update auxiliary parameter \lambda
    Bk=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama2=trace(Bk*E_i_R);
    
    alfa_kk=0.5*(E_v+nz);
    
    beta_kk=0.5*(E_v+gama2);
    
    E_lamda=alfa_kk/beta_kk;
    
    %%%%%%%Update scale matrix Pk1k
    ukk=u0+1;
    
    Ukk=U0+E_kasai*Ak;
    
    E_i_Pk1k=(ukk-nx-1)*inv(Ukk);
    
    %%%%%%%Update scale matrix R
    tkk=t0+1;
    
    Tkk=T0+E_lamda*Bk;
    
    E_i_R=(tkk-nz-1)*inv(Tkk);
    
    %%%%%%%Update dof parameter \omega
    akk=a+0.5;
    
    bkk=b+0.5*E_kasai+0.5-0.5*(psi(eta_kk)-log(theta_kk));
    
    E_w=akk/bkk;
    
    %%%%%%%Update dof parameter \nu
    ckk=c+0.5;
    
    dkk=d+0.5*E_lamda+0.5-0.5*(psi(alfa_kk)-log(beta_kk));
    
    E_v=ckk/dkk;

end

end

