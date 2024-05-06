%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));    
format long;

%%%%%Model parameters%%%%%%%%%%%%%%%%
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
ts=100;

%%%%Parameter selections
U1=1000;     
U2=1000;     
p1=0.95;     
p2=0.95;     
N=50;        
v=5;         
tao_P=4;     

%%%%Calculate true noise covariance matrix
D_Q=p1*Q0+(1-p1)*U1*Q0;
D_R=p2*R0+(1-p2)*U2*R0;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%Initial parameters%%%%%%%%%%%
    x=[0;0;10;10];                        %%%True initial state
    P=diag([100 100 100 100]);            %%%Initial estimation error covariance 
    Skk=utchol(P);                        

    %%%%KF initialization                 (Kalman filter)
    xf=x+Skk*randn(nx,1);                 %%%initial state estimate
    Pf=P;

    %%%%KF (Optimal Kalman filter)        Use instantaneous noise covariance matrix
    xtf=xf;                 
    Ptf=P;

    %%%%STKF initialization
    xst=xf;
    Pst=Pf;

    %%%%RSTKF initialization
    xapiv=xf;
    Papiv=Pf;

    %%%%ASTKF initialization
    xast=xf;
    Past=Pf;
    tau=0.15;
    lambda=0.15;
    
    %%%%Save data
    xA=x;
    xfA=xf;
    xtfA=xtf;
    xstA=xst;
    xapivA=xapiv;
    xastA=xast;
    
    %%%%Calculate MSE
    t=0;
    mse_kf_1(1,t+1,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
    mse_kf_2(1,t+1,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
    
    mse_tkf_1(1,t+1,expt)=(xA(1,t+1)-xtfA(1,t+1))^2+(xA(2,t+1)-xtfA(2,t+1))^2;
    mse_tkf_2(1,t+1,expt)=(xA(3,t+1)-xtfA(3,t+1))^2+(xA(4,t+1)-xtfA(4,t+1))^2;

    mse_stkf_1(1,t+1,expt)=(xA(1,t+1)-xstA(1,t+1))^2+(xA(2,t+1)-xstA(2,t+1))^2;
    mse_stkf_2(1,t+1,expt)=(xA(3,t+1)-xstA(3,t+1))^2+(xA(4,t+1)-xstA(4,t+1))^2;
    
    mse_apivbkf_1(1,t+1,expt)=(xA(1,t+1)-xapivA(1,t+1))^2+(xA(2,t+1)-xapivA(2,t+1))^2;
    mse_apivbkf_2(1,t+1,expt)=(xA(3,t+1)-xapivA(3,t+1))^2+(xA(4,t+1)-xapivA(4,t+1))^2;

    mse_astkf_1(1,t+1,expt)=(xA(1,t+1)-xastA(1,t+1))^2+(xA(2,t+1)-xastA(2,t+1))^2;
    mse_astkf_2(1,t+1,expt)=(xA(3,t+1)-xastA(3,t+1))^2+(xA(4,t+1)-xastA(4,t+1))^2;
    
    for t=1:ts
        
        %%%%Simulate true state and measurement
        test1=rand;
        test2=rand;
        
        if test1<=p1
            Q=Q0;
        else
            Q=U1*Q0;
        end

        if test2<=p2
            R=R0;
        else
            R=U2*R0;
        end
        
        %%%%
        SQ=utchol(Q);    
        SR=utchol(R);    
        
        %%%%
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);

        %%%%Filtering
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);
        
        [xtf,Ptf]=kf(xtf,Ptf,F,H,z,Q,R);

        [xst,Pst]=stkf(xst,Pst,F,H,z,Q0,R0,v);

        [xapiv,Papiv]=apivbkf_1(xapiv,Papiv,F,H,z,Q0,R0,N,v,v,tao_P);

        [xast,Past]=new_stkf(xast,Past,F,H,z,Q0,R0,v,N,tau,lambda);

        %%%%Save data
        xA=[xA x];
        xfA=[xfA xf];
        xtfA=[xtfA xtf];
        xstA=[xstA xst];
        xapivA=[xapivA xapiv];
        xastA=[xastA xast];
        
        %%%%Calculate MSE
        mse_kf_1(1,t+1,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t+1,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
        
        mse_tkf_1(1,t+1,expt)=(xA(1,t+1)-xtfA(1,t+1))^2+(xA(2,t+1)-xtfA(2,t+1))^2;
        mse_tkf_2(1,t+1,expt)=(xA(3,t+1)-xtfA(3,t+1))^2+(xA(4,t+1)-xtfA(4,t+1))^2;

        mse_stkf_1(1,t+1,expt)=(xA(1,t+1)-xstA(1,t+1))^2+(xA(2,t+1)-xstA(2,t+1))^2;
        mse_stkf_2(1,t+1,expt)=(xA(3,t+1)-xstA(3,t+1))^2+(xA(4,t+1)-xstA(4,t+1))^2;

        mse_apivbkf_1(1,t+1,expt)=(xA(1,t+1)-xapivA(1,t+1))^2+(xA(2,t+1)-xapivA(2,t+1))^2;
        mse_apivbkf_2(1,t+1,expt)=(xA(3,t+1)-xapivA(3,t+1))^2+(xA(4,t+1)-xapivA(4,t+1))^2;

        mse_astkf_1(1,t+1,expt)=(xA(1,t+1)-xastA(1,t+1))^2+(xA(2,t+1)-xastA(2,t+1))^2;
        mse_astkf_2(1,t+1,expt)=(xA(3,t+1)-xastA(3,t+1))^2+(xA(4,t+1)-xastA(4,t+1))^2;
        
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%Calculate RMSE
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_tkf_1=sqrt(mean(mse_tkf_1,3));
rmse_tkf_2=sqrt(mean(mse_tkf_2,3));

rmse_stkf_1=sqrt(mean(mse_stkf_1,3));
rmse_stkf_2=sqrt(mean(mse_stkf_2,3));

rmse_apivbkf_1=sqrt(mean(mse_apivbkf_1,3));
rmse_apivbkf_2=sqrt(mean(mse_apivbkf_2,3));

rmse_astkf_1=sqrt(mean(mse_astkf_1,3));
rmse_astkf_2=sqrt(mean(mse_astkf_2,3));

%%%%%%%Plot RMSE
figure;
j=0:ts;
plot(j*T,rmse_kf_1(1,:),'-k',j*T,rmse_tkf_1(1,:),'-*k',j*T,rmse_stkf_1(1,:),'-m',j*T,rmse_apivbkf_1(1,:),'-b',j*T,rmse_astkf_1(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
legend('KFTNCM','OKF','STF','RSTKF','Proposed ASTF');

figure;
j=0:ts;
plot(j*T,rmse_kf_2(1,:),'-k',j*T,rmse_tkf_2(1,:),'-*k',j*T,rmse_stkf_2(1,:),'-m',j*T,rmse_apivbkf_2(1,:),'-b',j*T,rmse_astkf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('KFTNCM','OKF','STF','RSTKF','Proposed ASTF');