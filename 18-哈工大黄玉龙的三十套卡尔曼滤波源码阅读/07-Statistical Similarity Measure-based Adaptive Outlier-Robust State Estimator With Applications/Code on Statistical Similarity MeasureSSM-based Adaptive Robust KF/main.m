%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab demo code for the paper
% M. Bai, Y. Huang, Y. Zhang and J. Chambers, "Statistical Similarity Measure-based
% Adaptive Outlier-Robust State Estimator With Applications"  
% IIEEE Transactions on Automatic Control.
% URL: https://ieeexplore.ieee.org/document/9779987

% If you use our code in your publication, please cite the above paper.
% Demo code written by Mingming Bai.
% Email: mingming.bai@hrbeu.edu.cn
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%准备工作%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%%模型参数%%%%%%%%%%%%%%%%
nxp=1000;
nx=4;
nz=2;
T=1;        
q=0.5; 
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=200;

%%%%重要参数选择
N=50;              %%%%%%最大变分迭代次数
gama=1.345;        %%%%%%调节参数
sigma=5;           %%%%%%高斯核带宽
v_sq=5;            %%%%%%自由度参数
tao_P=2;
tao_R=2;
epsilon=1e-8;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%系统初值设置%%%%%%%%%%%
    x=[0;0;10;10];                        %%%真实状态初值
    P=diag([10000 10000 100 100]);        %%%初始估计误差方差矩阵
    Skk=utchol(P);                        %%%初始估计误差协方差矩阵的方根

    %%%%标准KF初值      (Kalman filter)
    xf=x+Skk*randn(nx,1);                 %%%状态估计初值
    Pf=P;

    %%%%SSMKF_1初值     flag=3  IEEE TAC Huang
    xssm_13=xf;
    Pssm_13=Pf;
    
    %%%%SSMKF_2初值     flag=3  提出方法 AORKF
    xssm_23=xf;
    Pssm_23=Pf;

    %%%%数据存储
    xA=x;
    xfA=xf;
    xssm_13A=xssm_13;
    xssm_23A=xssm_23;
    
    for t=1:ts
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%野值控制参数
        U1=1000;        %%%%%%系统野值放大倍数 
        U2=1000;        %%%%%%量测野值放大倍数 
        p1=0.95;        %%%%%%系统正常值概率    
        p2=0.90;        %%%%%%量测正常值概率   
        
        %%%%等效的系统噪声和量测噪声方差
        D_Q=p1*Q0+(1-p1)*U1*Q0;
        D_R=p2*R0+(1-p2)*U2*R0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%仿真厚尾状态噪声   高斯混合
        test1=rand;
        if test1<=p1
            Q=Q0;
        else
            Q=U1*Q0;
        end
        SQ=utchol(Q);  
        vx=SQ*randn(nx,1);
        
        %%%%仿真厚尾量测噪声   高斯混合  相同野值特性
        test2=rand;
        if test2<=p2
            R=R0;
        else
            R=U2*R0;
        end
        %%%%
        SR=utchol(R);
        vz=SR*randn(nz,1);

        %%%%仿真真实的状态和量测
        x=F*x+vx;
        z=H*x+vz;
        
        %%%%调用滤波程序
        %%%%现有滤波方法
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);
        
        [xssm_13,Pssm_13]=ssmkf(xssm_13,Pssm_13,F,H,z,Q0,R0,sigma,v_sq,N,epsilon,3);

        %%%%提出滤波方法 AORKF
        [xssm_23,Pssm_23]=aorkf(xssm_23,Pssm_23,F,H,z,Q0,R0,sigma,v_sq,tao_P,tao_R,N,epsilon,3);
        
        %%%%数据存储
        xA=[xA x];
        xfA=[xfA xf];
        xssm_13A=[xssm_13A xssm_13];
        xssm_23A=[xssm_23A xssm_23];

        %%%%MSE计算  现有方法
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
        
        mse_ssmkf_13_1(1,t,expt)=(xA(1,t+1)-xssm_13A(1,t+1))^2+(xA(2,t+1)-xssm_13A(2,t+1))^2;
        mse_ssmkf_13_2(1,t,expt)=(xA(3,t+1)-xssm_13A(3,t+1))^2+(xA(4,t+1)-xssm_13A(4,t+1))^2;

        %%%%MSE计算  提出方法 AORKF
        mse_ssmkf_23_1(1,t,expt)=(xA(1,t+1)-xssm_23A(1,t+1))^2+(xA(2,t+1)-xssm_23A(2,t+1))^2;
        mse_ssmkf_23_2(1,t,expt)=(xA(3,t+1)-xssm_23A(3,t+1))^2+(xA(4,t+1)-xssm_23A(4,t+1))^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE计算  现有方法
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_ssmkf_13_1=sqrt(mean(mse_ssmkf_13_1,3));
rmse_ssmkf_13_2=sqrt(mean(mse_ssmkf_13_2,3));

%%%%%%%%%RMSE计算  提出方法 AORKF
rmse_ssmkf_23_1=sqrt(mean(mse_ssmkf_23_1,3));
rmse_ssmkf_23_2=sqrt(mean(mse_ssmkf_23_2,3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%平滑处理
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%平滑处理
%%%%%%%%%SRMSE计算  现有方法
srmse_kf_1=smooth(rmse_kf_1,10);
srmse_kf_2=smooth(rmse_kf_2,10);

srmse_ssmkf_13_1=smooth(rmse_ssmkf_13_1,10);
srmse_ssmkf_13_2=smooth(rmse_ssmkf_13_2,10);

%%%%%%%%%SRMSE计算  提出方法 AORKF
srmse_ssmkf_23_1=smooth(rmse_ssmkf_23_1,10);
srmse_ssmkf_23_2=smooth(rmse_ssmkf_23_2,10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%SRMSE曲线  现有方法与提出方法对比
figure;
subplot(211);
j=1:ts;
plot(j*T,srmse_kf_1,'-k','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_13_1,'-c',j*T,srmse_ssmkf_23_1,'-r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{pos} (m)');
axis tight;

subplot(212);
j=1:ts;
plot(j*T,srmse_kf_2,'-k','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_13_2,'-c',j*T,srmse_ssmkf_23_2,'-r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{vel} (m/s)');
legend('KFTNCM','SSMKF-sqrt','Proposed AORSE-sqrt');
axis tight;


