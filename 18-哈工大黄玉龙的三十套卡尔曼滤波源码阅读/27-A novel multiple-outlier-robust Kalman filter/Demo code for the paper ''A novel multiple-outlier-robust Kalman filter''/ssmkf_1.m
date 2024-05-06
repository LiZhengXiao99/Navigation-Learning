function [xkk,Pkk,lamda1,lamda2]=ssmkf_1(xkk,Pkk,F,H,z,Q,R,sigma,v,N,flag)

%%%%%%准备
nz=size(z,1);

nx=size(xkk,1);

%%%%%时间更新
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;
%%%%%量测更新
xkk=xk1k;

Pkk=Pk1k;

lamda1=1;

lamda2=1;

for i=1:N
    
    %%%%%%%
    xkk_i=xkk;
        
    %%%%%%%计算状态估计
    D_Pk1k=Pk1k/lamda1;

    D_R=R/lamda2;

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
    
    %%%%%%%计算辅助参数
    Dk1=(xkk-xk1k)*(xkk-xk1k)'+Pkk;
    
    Dk2=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';
    
    gama1=trace(Dk1*inv(Pk1k));

    gama2=trace(Dk2*inv(R));
    
    %%%%%%%第一种选择
    if flag==1
        lamda1=exp(0.5*(nx-gama1)/sigma^2);        
        lamda2=exp(0.5*(nz-gama2)/sigma^2);
    end
    
    %%%%%%%第二种选择
    if flag==2
        lamda1=(v+nx)/(v+gama1);
        lamda2=(v+nz)/(v+gama2);
    end

    %%%%%%%第三种选择
    if flag==3
        lamda1=sqrt((v+nx)/(v+gama1));
        lamda2=sqrt((v+nz)/(v+gama2));
    end

    %%%%%%%防止奇异
    if lamda1<1e-8
        lamda1=lamda1+1e-8;
    end
    %%%%%%%防止奇异
    if lamda2<1e-8
        lamda2=lamda2+1e-8;
    end
    
end



