%% Test run time
profile on

%% 
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%% Load data
load('data\imu.mat');

%% Model parameters
nxp=1000;
nx=4;
nz=2;
T=1;        
q=1;          
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=200;

%% Filtering parameters
N=50;        %% Maximum number of iterations
gama=1.345;  %% Tune parameter for HKF
sigma=11;    %% Gaussian kernel
v=5;         %% Dof parameter 
v_st=10;     %% Dof parameter 
v_sr=5;      %% Dof parameter 

%% Outlier parameters
U1=100;          
U2=500;          
p1=0.95;         
p2=0.95;         

%% True process and measurement noise covariance matrices
D_Q=p1*Q0+(1-p1)*U1*Q0;
D_R=p2*R0+(1-p2)*U2*R0;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %% Initialization
    x=[0;0;10;10];                        %%%真实状态初值
    P=diag([10000 10000 100 100]);        %%%初始估计误差方差矩阵
    Skk=utchol(P);                        %%%初始估计误差协方差矩阵的方根

    %% Kalman filter
    xf=x+Skk*randn(nx,1);                 %%%状态估计初值
    Pf=P;
    
    %% Fixed-point iterative algorithm
    %% SSMKF_1     flag=1 
    xssm_11=xf;
    Pssm_11=Pf;
    
    %% SSMKF_1     flag=2
    xssm_12=xf;
    Pssm_12=Pf;
    
    %% SSMKF_1     flag=3
    xssm_13=xf;
    Pssm_13=Pf;
    
    %% Separate iterative algorithm
    %% New_SSMKF_1     flag=1 
    xssm_21=xf;
    Pssm_21=Pf;
    
    %% New_SSMKF_1     flag=2 
    xssm_22=xf;
    Pssm_22=Pf;
    
    %% New_SSMKF_1     flag=3
    xssm_23=xf;
    Pssm_23=Pf;

    %% Save data
    xfA=xf;
    xssm_11A=xssm_11;
    xssm_12A=xssm_12;
    xssm_13A=xssm_13;
    xssm_21A=xssm_21;
    xssm_22A=xssm_22;
    xssm_23A=xssm_23;
    
    %% Extract data
    xA=XA(:,:,expt);
    zA=ZA(:,:,expt);
    
    for t=1:ts

        %% Extract measurement
        z=zA(:,t);
        
        %% Filtering
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);

        %% SSMKF_1 
        [xssm_11,Pssm_11]=ssmkf_1(xssm_11,Pssm_11,F,H,z,Q0,R0,sigma,v,N,1);
        
        [xssm_12,Pssm_12]=ssmkf_1(xssm_12,Pssm_12,F,H,z,Q0,R0,sigma,v_st,N,2);
        
        [xssm_13,Pssm_13]=ssmkf_1(xssm_13,Pssm_13,F,H,z,Q0,R0,sigma,v_sr,N,3);
        
        %% New_SSMKF_1
        [xssm_21,Pssm_21]=new_ssmkf_1(xssm_21,Pssm_21,F,H,z,Q0,R0,sigma,v,N,1);
        
        [xssm_22,Pssm_22]=new_ssmkf_1(xssm_22,Pssm_22,F,H,z,Q0,R0,sigma,v_st,N,2);
        
        [xssm_23,Pssm_23]=new_ssmkf_1(xssm_23,Pssm_23,F,H,z,Q0,R0,sigma,v_sr,N,3);

        %% 
        xfA=[xfA xf];
        xssm_11A=[xssm_11A xssm_11];
        xssm_12A=[xssm_12A xssm_12];
        xssm_13A=[xssm_13A xssm_13];
        xssm_21A=[xssm_21A xssm_21];
        xssm_22A=[xssm_22A xssm_22];
        xssm_23A=[xssm_23A xssm_23];

        %% Calculate MSE
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;

        %% SSMKF_1
        mse_ssmkf_11_1(1,t,expt)=(xA(1,t+1)-xssm_11A(1,t+1))^2+(xA(2,t+1)-xssm_11A(2,t+1))^2;
        mse_ssmkf_11_2(1,t,expt)=(xA(3,t+1)-xssm_11A(3,t+1))^2+(xA(4,t+1)-xssm_11A(4,t+1))^2;
        
        mse_ssmkf_12_1(1,t,expt)=(xA(1,t+1)-xssm_12A(1,t+1))^2+(xA(2,t+1)-xssm_12A(2,t+1))^2;
        mse_ssmkf_12_2(1,t,expt)=(xA(3,t+1)-xssm_12A(3,t+1))^2+(xA(4,t+1)-xssm_12A(4,t+1))^2;
        
        mse_ssmkf_13_1(1,t,expt)=(xA(1,t+1)-xssm_13A(1,t+1))^2+(xA(2,t+1)-xssm_13A(2,t+1))^2;
        mse_ssmkf_13_2(1,t,expt)=(xA(3,t+1)-xssm_13A(3,t+1))^2+(xA(4,t+1)-xssm_13A(4,t+1))^2;

        %% New_SSMKF_1
        mse_ssmkf_21_1(1,t,expt)=(xA(1,t+1)-xssm_21A(1,t+1))^2+(xA(2,t+1)-xssm_21A(2,t+1))^2;
        mse_ssmkf_21_2(1,t,expt)=(xA(3,t+1)-xssm_21A(3,t+1))^2+(xA(4,t+1)-xssm_21A(4,t+1))^2;
        
        mse_ssmkf_22_1(1,t,expt)=(xA(1,t+1)-xssm_22A(1,t+1))^2+(xA(2,t+1)-xssm_22A(2,t+1))^2;
        mse_ssmkf_22_2(1,t,expt)=(xA(3,t+1)-xssm_22A(3,t+1))^2+(xA(4,t+1)-xssm_22A(4,t+1))^2;
        
        mse_ssmkf_23_1(1,t,expt)=(xA(1,t+1)-xssm_23A(1,t+1))^2+(xA(2,t+1)-xssm_23A(2,t+1))^2;
        mse_ssmkf_23_2(1,t,expt)=(xA(3,t+1)-xssm_23A(3,t+1))^2+(xA(4,t+1)-xssm_23A(4,t+1))^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%% Calculate RMSE
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

%% SSMKF_1
rmse_ssmkf_11_1=sqrt(mean(mse_ssmkf_11_1,3));
rmse_ssmkf_11_2=sqrt(mean(mse_ssmkf_11_2,3));

rmse_ssmkf_12_1=sqrt(mean(mse_ssmkf_12_1,3));
rmse_ssmkf_12_2=sqrt(mean(mse_ssmkf_12_2,3));

rmse_ssmkf_13_1=sqrt(mean(mse_ssmkf_13_1,3));
rmse_ssmkf_13_2=sqrt(mean(mse_ssmkf_13_2,3));

%% New_SSMKF_1
rmse_ssmkf_21_1=sqrt(mean(mse_ssmkf_21_1,3));
rmse_ssmkf_21_2=sqrt(mean(mse_ssmkf_21_2,3));
 
rmse_ssmkf_22_1=sqrt(mean(mse_ssmkf_22_1,3));
rmse_ssmkf_22_2=sqrt(mean(mse_ssmkf_22_2,3));
 
rmse_ssmkf_23_1=sqrt(mean(mse_ssmkf_23_1,3));
rmse_ssmkf_23_2=sqrt(mean(mse_ssmkf_23_2,3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%% Calculate ARMSE 
armse_kf_1=mean(rmse_kf_1(40:end))
armse_kf_2=mean(rmse_kf_2(40:end))

%% SSMKF_1
armse_ssmkf_11_1=mean(rmse_ssmkf_11_1(40:end))
armse_ssmkf_11_2=mean(rmse_ssmkf_11_2(40:end))

armse_ssmkf_12_1=mean(rmse_ssmkf_12_1(40:end))
armse_ssmkf_12_2=mean(rmse_ssmkf_12_2(40:end))

armse_ssmkf_13_1=mean(rmse_ssmkf_13_1(40:end))
armse_ssmkf_13_2=mean(rmse_ssmkf_13_2(40:end))

%% New_SSMKF_1
armse_ssmkf_21_1=mean(rmse_ssmkf_21_1(40:end))
armse_ssmkf_21_2=mean(rmse_ssmkf_21_2(40:end))

armse_ssmkf_22_1=mean(rmse_ssmkf_22_1(40:end))
armse_ssmkf_22_2=mean(rmse_ssmkf_22_2(40:end))

armse_ssmkf_23_1=mean(rmse_ssmkf_23_1(40:end))
armse_ssmkf_23_2=mean(rmse_ssmkf_23_2(40:end))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Smoothing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculate SRMSE
srmse_kf_1=smooth(rmse_kf_1,10);
srmse_kf_2=smooth(rmse_kf_2,10);

%% SSMKF_1
srmse_ssmkf_11_1=smooth(rmse_ssmkf_11_1,10);
srmse_ssmkf_11_2=smooth(rmse_ssmkf_11_2,10);

srmse_ssmkf_12_1=smooth(rmse_ssmkf_12_1,10);
srmse_ssmkf_12_2=smooth(rmse_ssmkf_12_2,10);

srmse_ssmkf_13_1=smooth(rmse_ssmkf_13_1,10);
srmse_ssmkf_13_2=smooth(rmse_ssmkf_13_2,10);

%% New_SSMKF_1
srmse_ssmkf_21_1=smooth(rmse_ssmkf_21_1,10);
srmse_ssmkf_21_2=smooth(rmse_ssmkf_21_2,10);

srmse_ssmkf_22_1=smooth(rmse_ssmkf_22_1,10);
srmse_ssmkf_22_2=smooth(rmse_ssmkf_22_2,10);

srmse_ssmkf_23_1=smooth(rmse_ssmkf_23_1,10);
srmse_ssmkf_23_2=smooth(rmse_ssmkf_23_2,10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% Compare SSMKF_1 and New_SSMKF_1
figure;
j=1:ts;
subplot(2,1,1);
plot(j*T,srmse_ssmkf_11_1,'-g',j*T,srmse_ssmkf_12_1,'-c',j*T,srmse_ssmkf_13_1,'-r','linewidth',2.5)
hold on;
plot(j*T,srmse_ssmkf_21_1,'--g',j*T,srmse_ssmkf_22_1,'--c',j*T,srmse_ssmkf_23_1,'--r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{pos} (m)');
subplot(2,1,2);
plot(j*T,srmse_ssmkf_11_2,'-g',j*T,srmse_ssmkf_12_2,'-c',j*T,srmse_ssmkf_13_2,'-r','linewidth',2.5)
hold on;
plot(j*T,srmse_ssmkf_21_2,'--g',j*T,srmse_ssmkf_22_2,'--c',j*T,srmse_ssmkf_23_2,'--r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{vel} (m/s)');
legend('SSMKF-exp-F','SSMKF-log-F','SSMKF-sqrt-F','SSMKF-exp-S','SSMKF-log-S','SSMKF-sqrt-S');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Note that the SSMKF_1 with flag=2 is identical to RSTKF
%% Compare existing methods and New_SSMKF_1
figure;
j=1:ts;
subplot(2,1,1);
plot(j*T,srmse_kf_1,'-k',j*T,srmse_ssmkf_12_1,'-c','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_21_1,'--g',j*T,srmse_ssmkf_22_1,'--c',j*T,srmse_ssmkf_23_1,'--r','linewidth',2.5);
xlabel('Time (s)');
ylabel('SRMSE_{pos} (m)');
subplot(2,1,2);
plot(j*T,srmse_kf_2,'-k',j*T,srmse_ssmkf_12_2,'-c','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_21_2,'--g',j*T,srmse_ssmkf_22_2,'--c',j*T,srmse_ssmkf_23_2,'--r','linewidth',2.5);
xlabel('Time (s)');
ylabel('SRMSE_{vel} (m/s)');
legend('KFTCM','RSTKF','Proposed SSMKF-exp-S','Proposed SSMKF-log-S','Proposed SSMKF-sqrt-S');

%%%%%
profile viewer