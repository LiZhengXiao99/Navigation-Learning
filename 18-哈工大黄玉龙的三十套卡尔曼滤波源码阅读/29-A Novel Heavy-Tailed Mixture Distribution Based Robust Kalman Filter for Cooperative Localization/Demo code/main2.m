%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A simulation demo for algorithm in the paper
%
%M. Bai, Y. Huang, Y. Zhang and F. Chen, "A Novel Heavy-Tailed Mixture Distribution Based Robust Kalman Filter for Cooperative Localization,"
%IEEE Transactions on Industrial Informatics, vol. 17, no. 5, pp. 3671-3681, May 2021.
%
%If you use our code in your publication, please cite our paper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%Set up
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%Model parameter
nxp=30;
nx=4;
nz=2;
T=1;
qa=0.1;
r=10;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q1=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*qa;  %%nominal parameter
R1=r*eye(2);  %%nominal parameter
ts=1000;

%%%mixture paprameters
M=12; 
q(1)=1;
for j=2:M
    q(j)=5*(j-1);
end

for j=1:M
    Qk(nx*(j-1)+1:nx*j,:)=Q1*q(j);  
end 

%%%%The choices of important parameters
U1=100;      
U2=100;      
p1=0.95;     
p2=0.90;     
N=20;        
v=5;         
tao_P=5;     
tao_R=5; 

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%Initial values
    x=[0;0;0;0];                          
    P=diag([10000 10000 100 100]);        
    Skk=utchol(P);                        

    %%%%Initial state
    xf=x+Skk*randn(nx,1);                 
    Pf=P;
    
    %%%%Real KF
    xreal=xf;
    Preal=Pf;

    %%%%The ivbkf
    xiv=xf;
    Piv=Pf;
    a=5;c=5;
    b=1;d=1;
    
    %%%%The proposed filter
    xastf=xf;
    Pastf=Pf;
    uastf=7;
    Uastf=uastf*R1;
    pik=5;
    rou=1-exp(-5);
    alphaastf=ones(1,M);
    
    %%%%Save data
    xA=x;
    xfA=xf;
    xivA=xiv;
    xastfA=xastf;
    xrealA=xreal;
    
    for t=1:ts
        
        %%%%Simulate true state and measurement
        test1=rand;
        test2=rand;
        
        if test1<=p1
            Qt=Q1;
        else
            Qt=U1*Q1;
        end

        if test2<=p2
            Rt=R1;
        else
            Rt=U2*R1;
        end
        
        %%%%Simulate the non-stationary heavy-tailed noises 
        if t>=1 && t<=300
            Q=20*Qt;
            R=10*Rt;

        elseif t>=301 && t<=700
            Q=50*Qt;
            R=15*Rt;

        elseif t>=701 && t<=1000
            Q=20*Qt;
            R=10*Rt;

        end
        
        %%%%Calculate square-root matrix
        SQ=utchol(Q);    
        SR=utchol(R);    
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);
        
        %%%The KF with real noise covariance mxtrix
        [xreal,Preal]=kf(xreal,Preal,F,H,z,Q,R);

        [xiv,Piv]=aprwvivbkf(xiv,Piv,F,H,z,Q1,R1,tao_P,tao_R,a,b,c,d,N);
  
        [xastf,Pastf,alphaastf,uastf,Uastf]...
            =apivbstf(xastf,Pastf,F,H,z,Qk,R1,alphaastf,uastf,Uastf,pik,rou,M,N,v,v);

        %%%%Save data
        xA=[xA x];
        xivA=[xivA xiv];
        xastfA=[xastfA xastf];
        xrealA=[xrealA xreal];
        
        %%%%MSE calculation
        mse_ivbkf_1(1,t,expt)=(xA(1,t+1)-xivA(1,t+1))^2+(xA(2,t+1)-xivA(2,t+1))^2;
        mse_ivbkf_2(1,t,expt)=(xA(3,t+1)-xivA(3,t+1))^2+(xA(4,t+1)-xivA(4,t+1))^2;
        
        mse_apivbstf_1(1,t,expt)=(xA(1,t+1)-xastfA(1,t+1))^2+(xA(2,t+1)-xastfA(2,t+1))^2;
        mse_apivbstf_2(1,t,expt)=(xA(3,t+1)-xastfA(3,t+1))^2+(xA(4,t+1)-xastfA(4,t+1))^2;
        
        mse_kft_1(1,t,expt)=(xA(1,t+1)-xrealA(1,t+1))^2+(xA(2,t+1)-xrealA(2,t+1))^2;
        mse_kft_2(1,t,expt)=(xA(3,t+1)-xrealA(3,t+1))^2+(xA(4,t+1)-xrealA(4,t+1))^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE calculations

rmse_ivbkf_1=sqrt(mean(mse_ivbkf_1,3));
rmse_ivbkf_2=sqrt(mean(mse_ivbkf_2,3));

rmse_apivbstf_1=sqrt(mean(mse_apivbstf_1,3));
rmse_apivbstf_2=sqrt(mean(mse_apivbstf_2,3));

rmse_kft_1=sqrt(mean(mse_kft_1,3));
rmse_kft_2=sqrt(mean(mse_kft_2,3));

%%%%%%%%%%%%%%%%%%%%%Smooth%%%%%%%%%%%%%%%%%%%%%
%%%%%%%RMSE smooth
rmse_kft_1=smooth(rmse_kft_1,10);
rmse_kft_2=smooth(rmse_kft_2,10);

rmse_ivbkf_1=smooth(rmse_ivbkf_1,10);
rmse_ivbkf_2=smooth(rmse_ivbkf_2,10);

rmse_apivbstf_1=smooth(rmse_apivbstf_1,10);
rmse_apivbstf_2=smooth(rmse_apivbstf_2,10);


%%%%%%%smooth RMSE curve
figure;
j = 1:ts;
plot(j*T,rmse_kft_1,'-k',j*T,rmse_ivbkf_1,'-b',j*T,rmse_apivbstf_1,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
legend('OKF','STKF-\Sigma R & dof','The proposed filter');

figure;
j = 1:ts;
plot(j*T,rmse_kft_2,'-k',j*T,rmse_ivbkf_2,'-b',j*T,rmse_apivbstf_2,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('OKF','STKF-\Sigma R & dof','The proposed filter');