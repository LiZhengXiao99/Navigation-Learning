%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Filter in the paper
%Y. Huang, Y. Zhang, B. Xu, Z. Wu and J. Chambers, "A New Outlier-Robust Student's t Based Gaussian Approximate Filter for Cooperative Localization,"
%IEEE/ASME Transactions on Mechatronics, vol. 22, no. 5, pp. 2380-2386, Oct. 2017, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xkk,Pkk,D_Pk1k,D_R]=aprwvivbkf(xkk,Pkk,F,H,z,Q,R,tao_p,tao_r,a,b,c,d,N)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
%%%%%变分迭代初值
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
    
    %%%%%更新状态
    %%%%%计算修正的Pk1k和R
    D_Pk1k=inv(E_i_Pk1k)/E_kasai;
    
    D_Pk1k=(D_Pk1k+D_Pk1k')/2;
    
    D_R=inv(E_i_R)/E_lamda;
    
    %%%%%更新状态
    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%更新kasai的分布
    Ak=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    gama1=trace(Ak*E_i_Pk1k);
    
    eta_kk=0.5*(E_w+nx);
    
    theta_kk=0.5*(E_w+gama1);
    
    E_kasai=eta_kk/theta_kk;
    
    %%%%%%%更新lamda的分布
    Bk=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama2=trace(Bk*E_i_R);
    
    alfa_kk=0.5*(E_v+nz);
    
    beta_kk=0.5*(E_v+gama2);
    
    E_lamda=alfa_kk/beta_kk;
    
    %%%%%%%更新Pk1k的分布
    ukk=u0+1;
    
    Ukk=U0+E_kasai*Ak;
    
    E_i_Pk1k=(ukk-nx-1)*inv(Ukk);
    
    %%%%%%%更新R的分布
    tkk=t0+1;
    
    Tkk=T0+E_lamda*Bk;
    
    E_i_Rk=inv(R);
    
    %%%%%%%更新自由度参数w的分布
    akk=a+0.5;
    
    bkk=b+0.5*E_kasai+0.5-0.5*(psi(eta_kk)-log(theta_kk));
    
    E_w=akk/bkk;
    
    %%%%%%%更新自由度参数v的分布
    ckk=c+0.5;
    
    dkk=d+0.5*E_lamda+0.5-0.5*(psi(alfa_kk)-log(beta_kk));
    
    E_v=ckk/dkk;

end

E_Pk1k=inv(E_i_Pk1k);

E_R=inv(E_i_R);

end

