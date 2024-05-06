%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));          
format long;

%%%%%Model parameters%%%%%%%
nxp=1000;
nx=4;
nz=2;
T=1;       
q=1;
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q1=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R1=r*[1 0.5;0.5 1];
ts=1000;

%%%%%Selections for important parameters
N=10;                 %%%%%%The number of variational iteration     
tao_P=3;   
tao_R=3;   
rou=1-exp(-4);
alfa=1;
beta=100;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%Initial values
    x=[100;100;10;10];                    
    P=diag([100 100 100 100]);            
    Skk=utchol(P);                        
    
    %%%%Nominal noise covariance matrices
    Q0=alfa*eye(nx);
    R0=beta*eye(nz);
    
    %%%%Kalman filter with nominal noise covariance matrices (KFNCM)
    xf=x+Skk*randn(nx,1);                 
    Pf=P;
    
    %%%%Kalman filter with true noise covariance matrices (KFTCM)
    xtf=xf;
    Ptf=Pf;
    
    %%%%Proposed ivbkf-PR
    xapriv=xf;
    Papriv=Pf;
    uapriv=(nz+1+tao_R);
    Uapriv=tao_R*R0;
    
    %%%%Save data
    xA=x;
    xfA=xf;
    xtfA=xtf;
    xaprivA=xapriv;
    
    for t=1:ts
        
        %%%%True noise covariance matrices
        Q=(6.5+0.5*cos(pi*t/ts))*Q1;
        R=(0.1+0.05*cos(pi*t/ts))*R1;
        
        %%%%Square-root of noise covariance matrices
        SQ=utchol(Q);    
        SR=utchol(R);    
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);

        %%%%Filtering
        [xf,Pf,Ppf]=kf(xf,Pf,F,H,z,Q0,R0);
        
        [xtf,Ptf,Pptf]=kf(xtf,Ptf,F,H,z,Q,R);

        [xapriv,Papriv,uapriv,Uapriv,Ppapriv,Rapriv]=aprivbkf(xapriv,Papriv,uapriv,Uapriv,F,H,z,Q0,R0,N,tao_P,rou);

        %%%%Save data
        xA=[xA x];
        xfA=[xfA xf];
        xtfA=[xtfA xtf];
        xaprivA=[xaprivA xapriv];
        
        %%%%MSE calculation
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
        
        mse_ktf_1(1,t,expt)=(xA(1,t+1)-xtfA(1,t+1))^2+(xA(2,t+1)-xtfA(2,t+1))^2;
        mse_ktf_2(1,t,expt)=(xA(3,t+1)-xtfA(3,t+1))^2+(xA(4,t+1)-xtfA(4,t+1))^2;

        mse_aprivbkf_1(1,t,expt)=(xA(1,t+1)-xaprivA(1,t+1))^2+(xA(2,t+1)-xaprivA(2,t+1))^2;
        mse_aprivbkf_2(1,t,expt)=(xA(3,t+1)-xaprivA(3,t+1))^2+(xA(4,t+1)-xaprivA(4,t+1))^2;
        
        %%%%Calculate the estimation error of Pk1k
        P_kf(1,t,expt)=norm(Ppf-Pptf,'fro')^2/nx^2;

        P_aprivbkf(1,t,expt)=norm(Ppapriv-Pptf,'fro')^2/nx^2;
        
        %%%%Calculate the estimation error of R
        R_kf(1,t,expt)=norm(R0-R,'fro')^2/nz^2;

        R_aprivbkf(1,t,expt)=norm(Rapriv-R,'fro')^2/nz^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE calculation
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_ktf_1=sqrt(mean(mse_ktf_1,3));
rmse_ktf_2=sqrt(mean(mse_ktf_2,3));

rmse_aprivbkf_1=sqrt(mean(mse_aprivbkf_1,3));
rmse_aprivbkf_2=sqrt(mean(mse_aprivbkf_2,3));

%%%%%%%RMSE curves
figure;
j = 1:ts;
subplot(2,1,1)
plot(j*T,rmse_kf_1(1,:),'-k',j*T,rmse_ktf_1(1,:),'-c',j*T,rmse_aprivbkf_1(1,:),'-r','linewidth',2.5);
ylabel('RMSE_{pos} (m)');
subplot(2,1,2)
plot(j*T,rmse_kf_2(1,:),'-k',j*T,rmse_ktf_2(1,:),'-c',j*T,rmse_aprivbkf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('KFNCM','KFTCM','The proposed filter');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%
NP_kf=sqrt(sqrt(mean(P_kf,3)));

NP_aprivbkf=sqrt(sqrt(mean(P_aprivbkf,3)));

%%%%%%%%%
NR_kf=sqrt(sqrt(mean(R_kf,3)));

NR_aprivbkf=sqrt(sqrt(mean(R_aprivbkf,3)));

%%%%%%%
figure;
j = 1:ts;
subplot(2,1,1)
plot(j*T,NP_kf(1,:),'-k',j*T,NP_aprivbkf(1,:),'-r','linewidth',2.5)
ylabel('SRNFN of PECM');
subplot(2,1,2)
plot(j*T,NR_kf(1,:),'-k',j*T,NR_aprivbkf(1,:),'-r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRNFN of MNCM');
legend('KFNCM','The proposed filter');
