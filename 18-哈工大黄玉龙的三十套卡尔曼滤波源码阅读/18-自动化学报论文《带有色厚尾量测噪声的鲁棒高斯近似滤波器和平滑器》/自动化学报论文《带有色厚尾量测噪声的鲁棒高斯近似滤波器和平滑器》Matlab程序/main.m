%%%%%准备工作%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     %%%设置发生器在每次的状态不相同
format long;

%%%%%模型参数%%%%%%%%%%%%%%%%
raddeg=pi/180;                     %%%度变弧度
nx=5;                              %%%状态维数
nz=2;                              %%%观测维数
nxp=200;                           %%%Monte Carlo simulation 次数
ts=100;                            
T=1;
M=[T^3/3 T^2/2;T^2/2 T];
%%%%噪声方差设置%%%%%%%%%%%%%%
q1=0.1;
q2=1.75e-4;
Cr=10;
Co=sqrt(10)*1e-3;
Q=[q1*M zeros(2,2) zeros(2,1);zeros(2,2) q1*M zeros(2,1);zeros(1,2) zeros(1,2) q2*T]; 
R0=diag([Cr^2 Co^2]);

%%%%相关参数
phi=[0.5 0;0 0.5];

%%%%正常值概率
p=0.95;

%%%%变分迭代次数
N=5;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%系统初值设置%%%%%%%%%%%
    x=[1000;300;1000;0;-3*raddeg];        %%%真实状态初值
    P=diag([100 10 100 10 1e-4]);         %%%初始估计误差方差矩阵 
    Skk=utchol(P);                        %%%初始估计误差协方差矩阵的方根
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%产生初始量测
    test=rand;
    if test<=p
        R=R0;
    else
        R=100*R0;
    end
    %%%%计算方根矩阵
    SR=utchol(R);
    v=SR*randn(nz,1);
    z=MstEq(x)+v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%改进CKS初值
    xi=x+Skk*randn(nx,1);                 %%%状态估计初值
    Pi=P;
    
    %%%%现有MCKS初值 
    xm=xi;
    Pm=Pi;
    
    %%%%现有标准有色CKS初值 
    xs=xi;
    Ps=Pi;
    
    %%%%改进CKF初值
    xif=xi;
    Pif=Pi;
    aif=5;
    bif=1;
    uif=nz+2;
    Uif=R0;
    
    %%%%现有MCKF初值
    xmf=xi;
    Pmf=Pi;
    
    %%%%现有标准有色CKF初值 
    xsf=xi;
    Psf=Pi;

    %%%%数据存储
    xA=x;
    zA=z;
    yA=[];
    xifA=xif;
    xmfA=xmf;
    xsfA=xsf;

    %%%%仿真真实的状态和量测
    for t=1:ts
        
        test=rand;

        if test<=p
            R=R0;
        else
            R=100*R0;
        end

        %%%%计算方根矩阵
        SQ=utchol(Q);    
        SR=utchol(R);
        
        %%%%仿真真实的状态和量测
        x=ProssEq(x)+SQ*randn(nx,1);
        %%%%产生有色后尾量测噪声
        v=phi*v+SR*randn(nz,1);
        %%%%产生量测
        z=MstEq(x)+v;
        
        %%%%产生白色量测
        y=z-phi*zA(:,t);
        
        %%%%调用滤波程序
        [xif,Pif,uif,Uif,aif,bif]=icckf(xif,Pif,y,Q,phi,uif,Uif,aif,bif,N);
        
        [xmf,Pmf]=orckf(xmf,Pmf,z,Q,R0,5,N); 
        
        [xsf,Psf,xsfkk1,Psfkk1,Ask]=cckf(xsf,Psf,y,Q,R0,phi);

        %%%%存储状态和量测
        xA=[xA x];  
        zA=[zA z];
        yA=[yA y];
        
        %%%%%滤波存储
        xifA=[xifA xif];  
        xmfA=[xmfA xmf];  
        xsfA=[xsfA xsf];  
        
    end

    %%%%改进CKS的初始参数
    a0=5;
    b0=1;
    u0=nz+2;
    U0=R0;
    %%%%提取量测
    zA=zA(:,2:end);
    
    %%%%调用平滑程序
    [ixsBA,iPsBA]=iccks(xi,Pi,yA,Q,phi,ts,a0,b0,u0,U0,N);
    
    [mxsBA,mPsBA]=mcks(xm,Pm,zA,ts,Q,R0,5,N);
    
    [sxsBA,sPsBA]=sccks(xs,Ps,yA,Q,phi,ts,R0);

    %%%%MSE计算
    %%%%%%滤波
    mse_ickf_1(expt,:)=(xA(1,:)-xifA(1,:)).^2+(xA(3,:)-xifA(3,:)).^2;
    mse_ickf_2(expt,:)=(xA(2,:)-xifA(2,:)).^2+(xA(4,:)-xifA(4,:)).^2;
    mse_ickf_3(expt,:)=(xA(5,:)-xifA(5,:)).^2;
    
    mse_mckf_1(expt,:)=(xA(1,:)-xmfA(1,:)).^2+(xA(3,:)-xmfA(3,:)).^2;
    mse_mckf_2(expt,:)=(xA(2,:)-xmfA(2,:)).^2+(xA(4,:)-xmfA(4,:)).^2;
    mse_mckf_3(expt,:)=(xA(5,:)-xmfA(5,:)).^2;
    
    mse_sckf_1(expt,:)=(xA(1,:)-xsfA(1,:)).^2+(xA(3,:)-xsfA(3,:)).^2;
    mse_sckf_2(expt,:)=(xA(2,:)-xsfA(2,:)).^2+(xA(4,:)-xsfA(4,:)).^2;
    mse_sckf_3(expt,:)=(xA(5,:)-xsfA(5,:)).^2;
    
    %%%%%平滑
    mse_icks_1(expt,:)=(xA(1,:)-ixsBA(1,:)).^2+(xA(3,:)-ixsBA(3,:)).^2;
    mse_icks_2(expt,:)=(xA(2,:)-ixsBA(2,:)).^2+(xA(4,:)-ixsBA(4,:)).^2;
    mse_icks_3(expt,:)=(xA(5,:)-ixsBA(5,:)).^2;
    
    mse_mcks_1(expt,:)=(xA(1,:)-mxsBA(1,:)).^2+(xA(3,:)-mxsBA(3,:)).^2;
    mse_mcks_2(expt,:)=(xA(2,:)-mxsBA(2,:)).^2+(xA(4,:)-mxsBA(4,:)).^2;
    mse_mcks_3(expt,:)=(xA(5,:)-mxsBA(5,:)).^2;
    
    mse_scks_1(expt,:)=(xA(1,:)-sxsBA(1,:)).^2+(xA(3,:)-sxsBA(3,:)).^2;
    mse_scks_2(expt,:)=(xA(2,:)-sxsBA(2,:)).^2+(xA(4,:)-sxsBA(4,:)).^2;
    mse_scks_3(expt,:)=(xA(5,:)-sxsBA(5,:)).^2;

end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%滤波
rmse_ickf_1=sqrt(mean(mse_ickf_1,1));
rmse_ickf_2=sqrt(mean(mse_ickf_2,1));
rmse_ickf_3=sqrt(mean(mse_ickf_3,1));

rmse_mckf_1=sqrt(mean(mse_mckf_1,1));
rmse_mckf_2=sqrt(mean(mse_mckf_2,1));
rmse_mckf_3=sqrt(mean(mse_mckf_3,1));

rmse_sckf_1=sqrt(mean(mse_sckf_1,1));
rmse_sckf_2=sqrt(mean(mse_sckf_2,1));
rmse_sckf_3=sqrt(mean(mse_sckf_3,1));

%%%%%%%平滑
rmse_icks_1=sqrt(mean(mse_icks_1,1));
rmse_icks_2=sqrt(mean(mse_icks_2,1));
rmse_icks_3=sqrt(mean(mse_icks_3,1));

rmse_mcks_1=sqrt(mean(mse_mcks_1,1));
rmse_mcks_2=sqrt(mean(mse_mcks_2,1));
rmse_mcks_3=sqrt(mean(mse_mcks_3,1));

rmse_scks_1=sqrt(mean(mse_scks_1,1));
rmse_scks_2=sqrt(mean(mse_scks_2,1));
rmse_scks_3=sqrt(mean(mse_scks_3,1));

%%%%%%%%%%%画图
figure;
j = 0:ts;
plot(j,rmse_sckf_1(1,:),'--g',j,rmse_mckf_1(1,:),'--b',j,rmse_ickf_1(1,:),'--r',j,rmse_scks_1(1,:),'-g',j,rmse_mcks_1(1,:),'-b',j,rmse_icks_1(1,:),'-r','linewidth',2.5);
xlabel('时间 (s)');
ylabel('位置的RMSE (m)');
legend('现有的有色CKF','现有的鲁棒CKF','提出的鲁棒CKF','现有的有色CKS','现有的鲁棒CKS','提出的鲁棒CKS');

figure;
j = 0:ts;
plot(j,rmse_sckf_2(1,:),'--g',j,rmse_mckf_2(1,:),'--b',j,rmse_ickf_2(1,:),'--r',j,rmse_scks_2(1,:),'-g',j,rmse_mcks_2(1,:),'-b',j,rmse_icks_2(1,:),'-r','linewidth',2.5);
xlabel('时间 (s)');
ylabel('速度的RMSE (m/s)');
legend('现有的有色CKF','现有的鲁棒CKF','提出的鲁棒CKF','现有的有色CKS','现有的鲁棒CKS','提出的鲁棒CKS');

figure;
j = 0:ts;
plot(j,rmse_sckf_3(1,:)./raddeg,'--g',j,rmse_mckf_3(1,:)./raddeg,'--b',j,rmse_ickf_3(1,:)./raddeg,'--r',j,rmse_scks_3(1,:)./raddeg,'-g',j,rmse_mcks_3(1,:)./raddeg,'-b',j,rmse_icks_3(1,:)./raddeg,'-r','linewidth',2.5);
xlabel('时间 (s)');
ylabel('转弯速率的RMSE (Deg/s)');
legend('现有的有色CKF','现有的鲁棒CKF','提出的鲁棒CKF','现有的有色CKS','现有的鲁棒CKS','提出的鲁棒CKS');


%%%%%%%%%%%%%%%%%%
armse_ickf_1=sqrt(mean(mean(mse_ickf_1,1)))
armse_ickf_2=sqrt(mean(mean(mse_ickf_2,1)))
armse_ickf_3=sqrt(mean(mean(mse_ickf_3,1)))./raddeg

armse_icks_1=sqrt(mean(mean(mse_icks_1,1)))
armse_icks_2=sqrt(mean(mean(mse_icks_2,1)))
armse_icks_3=sqrt(mean(mean(mse_icks_3,1)))./raddeg

armse_mckf_1=sqrt(mean(mean(mse_mckf_1,1)))
armse_mckf_2=sqrt(mean(mean(mse_mckf_2,1)))
armse_mckf_3=sqrt(mean(mean(mse_mckf_3,1)))./raddeg
 
armse_mcks_1=sqrt(mean(mean(mse_mcks_1,1)))
armse_mcks_2=sqrt(mean(mean(mse_mcks_2,1)))
armse_mcks_3=sqrt(mean(mean(mse_mcks_3,1)))./raddeg

armse_sckf_1=sqrt(mean(mean(mse_sckf_1,1)))
armse_sckf_2=sqrt(mean(mean(mse_sckf_2,1)))
armse_sckf_3=sqrt(mean(mean(mse_sckf_3,1)))./raddeg
 
armse_scks_1=sqrt(mean(mean(mse_scks_1,1)))
armse_scks_2=sqrt(mean(mean(mse_scks_2,1)))
armse_scks_3=sqrt(mean(mean(mse_scks_3,1)))./raddeg

