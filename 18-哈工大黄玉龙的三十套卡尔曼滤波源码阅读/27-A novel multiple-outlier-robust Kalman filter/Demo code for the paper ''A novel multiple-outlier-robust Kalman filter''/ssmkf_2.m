function [xkk,Pkk,Pu_x,Pu_z]=ssmkf_2(xkk,Pkk,F,H,z,Q,R,sigma,v,N,flag)

%%%%%%准备
nx=size(xkk,1);

nz=size(z,1);

%%%%%时间更新
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;
%%%%%量测更新
xkk=xk1k;

Pkk=Pk1k;

%%%%%计算方根
Sk1k=utchol(Pk1k);

SR=utchol(R);

I_Sk1k=inv(Sk1k);

I_SR=inv(SR);

Pu_x=eye(nx);

Pu_z=eye(nz);

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
      
    %%%%%%%计算IMCCKF状态估计
    D_Pk1k=Sk1k*inv(Pu_x)*Sk1k';

    D_R=SR*inv(Pu_z)*SR';

    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%%判定
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=1e-16
        break;
    end

    %%%%%%计算辅助参数
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    ex=diag(I_Sk1k*Dk1*I_Sk1k');
    
    ez=diag(I_SR*Dk2*I_SR');
    
    Pu_x=fun_pu_i(ex,sigma,v,flag);
    
    Pu_z=fun_pu_i(ez,sigma,v,flag);

end