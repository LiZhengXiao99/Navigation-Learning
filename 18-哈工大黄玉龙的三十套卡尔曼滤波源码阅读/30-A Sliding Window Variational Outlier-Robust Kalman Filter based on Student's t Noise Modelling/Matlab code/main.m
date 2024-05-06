%%%%% Please run main_data.m first to generate swrkf_data.mat, and then run this program
profile on

clear all;
close all;
clc;
randn('state',sum(100*clock));
format long;
load('swrkf_data.mat')

%%%%%Parameters of the model%%%%%%%%%%%%%%%%
mct=10; %Monte Carlo times
nxp=1:mct;
nx=4;
nz=2;
T=1;
q=.5;
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=5000;
qT=qA';
rT=rA';

%%%%Parameters of the filters
tao_R=5;
tao_Q=5;
epsilon=1e-8;
L=10;
rou=1-exp(-4);
omega=5;
nu=5;

for expt = nxp
    
    fprintf('MC Run in Process = %d\n',expt);
    
    %%%%%Initial state%%%%%%%%%%%
    x=[0;0;10;10];                                      %%%True value
    P=diag([10000 10000 100 100]);        %%%Initial estimation error covariance
    Skk=utchol(P);                                    %%%The square root of P
    
    %%%%Initial estimate and its covariance of the TKF
    xf=x+Skk*randn(nx,1);                        
    Pf=P;
    
    %%%%Initial estimate and its covariance of the NKF
    xnf=xf;
    Pnf=Pf;
    
    %%%%Initial estimate and its covariance of the proposed SWRKF
    xsw=xf;
    Psw=Pf;
        
    %%%%Temporarily stored variables of the proposed SWRKF
    zA=[];
    xsw_A=xsw;
    Psw_A=Psw;
    ysw_A=tao_Q;
    Ysw_A=tao_Q*Q0;
    usw_A=tao_R;
    Usw_A=tao_R*R0;
    Qsw=Q0;
    Rsw=R0;
    ksisw=[];
    lambdasw=[];

    %%%%Store variables for true state and estimated states of different
    %%%%filters
    xA=x;
    xfA=xf;
    xnfA=xnf;
    xswA=xsw;

    for t=1:ts
        x=XA(:,t,expt);
        z=ZA(:,t,expt);
        q=qA(t,expt);
        r=rA(t,expt);
        if q==1
            Q=Q0;
        else
            Q=U1*Q0;
        end
        if r==1
            R=R0;
        else
            R=U2*R0;
        end
        %%%%Temporarily stored measurements
        if t<=(L)
            zA=[zA z];%the length of zA is L
        else
            zA=[zA(:,2:end) z];
        end
        
        %%%%Filtering (Time update and measurement update)
        %TKF
        [xf,Pf]=kf(xf,Pf,F,H,z,Q,R); 
        %NKF
        [xnf,Pnf]=kf(xnf,Pnf,F,H,z,Q0,R0); 
        %Proposed SWRKF
        [xsw,Psw,xsw_A,Psw_A,xswnkNA,PswnkNA,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,ksisw,lambdasw,Nsw,M]=...
            swrkf(xsw_A,Psw_A,ysw_A,Ysw_A,usw_A,Usw_A,Qsw,Rsw,F,H,zA,ksisw,lambdasw,omega,nu,rou,L,t,2,epsilon);
        
        %%%%Store the data
        xA=[xA x];
        xfA=[xfA xf];
        xnfA=[xnfA xnf];
        xswA=[xswA xsw];
        
        %%%%Calculate the MSE
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
        
        mse_nf_1(1,t,expt)=(xA(1,t+1)-xnfA(1,t+1))^2+(xA(2,t+1)-xnfA(2,t+1))^2;
        mse_nf_2(1,t,expt)=(xA(3,t+1)-xnfA(3,t+1))^2+(xA(4,t+1)-xnfA(4,t+1))^2;
        
        mse_sw_1(1,t,expt)=(xA(1,t+1)-xswA(1,t+1))^2+(xA(2,t+1)-xswA(2,t+1))^2;
        mse_sw_2(1,t,expt)=(xA(3,t+1)-xswA(3,t+1))^2+(xA(4,t+1)-xswA(4,t+1))^2;

        ees_f(1,t,expt)=(x-xf)'*inv(Pf)*(x-xf);
        ees_nf(1,t,expt)=(x-xnf)'*inv(Pnf)*(x-xnf);
        ees_sw(1,t,expt)=(x-xsw)'*inv(Psw)*(x-xsw);
        
    end    
end

nees_f=mean(ees_f,3);
nees_nf=mean(ees_nf,3);
nees_sw=mean(ees_sw,3);

snees_f=smooth(nees_f,20);
snees_nf=smooth(nees_nf,20);
snees_sw=smooth(nees_sw,20);

anees_f=mean(nees_f);
anees_nf=mean(nees_nf);
anees_sw=mean(nees_sw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%Calculate the RMSE
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_nf_1=sqrt(mean(mse_nf_1,3));
rmse_nf_2=sqrt(mean(mse_nf_2,3));

rmse_sw_1=sqrt(mean(mse_sw_1,3));
rmse_sw_2=sqrt(mean(mse_sw_2,3));


armse_kf_1=mean(rmse_kf_1)
armse_kf_2=mean(rmse_kf_2)

armse_nf_1=mean(rmse_nf_1)
armse_nf_2=mean(rmse_nf_2)

armse_sw_1=mean(rmse_sw_1)
armse_sw_2=mean(rmse_sw_2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Smoothing the data
%%%%%%%%%Calculate the SRMSE
srmse_kf_1=smooth(rmse_kf_1,20);
srmse_kf_2=smooth(rmse_kf_2,20);

srmse_nf_1=smooth(rmse_nf_1,20);
srmse_nf_2=smooth(rmse_nf_2,20);

srmse_sw_1=smooth(rmse_sw_1,20);
srmse_sw_2=smooth(rmse_sw_2,20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%The curve of NEES
figure;
j=1:ts;
plot(j*T,repmat(nx,1,length(j)),'--','linewidth',6,'Color',[.3 .5 .2])
hold on;
plot(j*T,snees_f,'-k',j*T,snees_nf,'-y',j*T,snees_sw,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('NEES');
legend('Reference value','TKF','NKF','SWRKF');
axis tight;
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%The curve of the SRMSE
figure;
j=1:ts;
subplot(2,1,1)
plot(j*T,srmse_kf_1,'-k',j*T,srmse_nf_1,'-y',j*T,srmse_sw_1,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
axis tight;
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);
subplot(2,1,2)
plot(j*T,srmse_kf_2,'-k',j*T,srmse_nf_2,'-y',j*T,srmse_sw_2,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('TKF','NKF','SWRKF');
axis tight;
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);

%%%%%
profile viewer