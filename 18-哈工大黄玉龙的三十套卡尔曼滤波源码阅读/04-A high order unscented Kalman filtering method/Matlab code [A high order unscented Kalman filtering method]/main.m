%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%
nxp=1000;
T=100;
n=2;

%%%%Parameters for high-degree UKF
a2=n-1;
a1=2*n^2-14*n;
a0=n^3-13*n^2+60*n-60;
p=[a2 a1 a0];
kai=roots(p);
k1=kai(1);
k2=kai(2);
best_kai=k2;

for expt = 1:nxp

    fprintf('MC Run in Process = %d\n',expt); 

    %%%%
    Q=0.1*[1 0.5;0.5 1];
    SQ=utchol(Q);
    R=0.025;
    x= [20 5]';
    P=diag([0.1 0.1]);

    %%%%CKF
    xc=[20 5]'+SQ*randn(2,1);
    Pc=P;
    
    %%%%high_UKF
    high_xu=xc;
    high_Pu=P;

    %%%%Save data
    xA=x;
    xcA=xc;
    high_xuA=high_xu;

    for t=1:T
        
        x=ProssEq(x)+SQ*randn(n,1);  
        z=MstEq(x,t)+sqrt(R)*randn;
        
        %%%%Filtering
        [xc,Pc]=ckf(xc,Pc,z,Q,R,t);

        [high_xu,high_Pu] = high_ukf(high_xu,high_Pu,z,Q,R,t,best_kai);
        
        %%%%Save data
        xA=[xA x];
        xcA=[xcA xc];
        high_xuA=[high_xuA high_xu];
        
        %%%%Calculate MSE
        MSE_ckf_Pos(1,t,expt)=(xA(1,t)-xcA(1,t))^2;
        MSE_ckf_Vel(1,t,expt)=(xA(2,t)-xcA(2,t))^2;

        MSE_high_ukf_Pos(1,t,expt)=(xA(1,t)-high_xuA(1,t))^2;
        MSE_high_ukf_Vel(1,t,expt)=(xA(2,t)-high_xuA(2,t))^2;

    end;  
    
end;  

%%%%Calculate average MSE
MSE_ckf_Pos=mean(MSE_ckf_Pos,3);
MSE_ckf_Vel=mean(MSE_ckf_Vel,3);

MSE_high_ukf_Pos=mean(MSE_high_ukf_Pos,3);
MSE_high_ukf_Vel=mean(MSE_high_ukf_Vel,3);

%%%%MSE curves
figure;
j = 1:T;
plot(j,MSE_ckf_Pos(1,:),'-k',j,MSE_high_ukf_Pos(1,:),'-r','lineWidth',2.0);
legend('3rd-CKF','The proposed high-degree UKF(\kappa=0.835)');
xlabel('time(s)');
ylabel('MSE of x_1');
hold on;

figure;
j = 1:T;
plot(j,MSE_ckf_Vel(1,:),'-k',j,MSE_high_ukf_Vel(1,:),'-r','lineWidth',2.0);
legend('3rd-CKF','The proposed high-degree UKF(\kappa=0.835)');
xlabel('time(s)');
ylabel('MSE of x_2');
hold on;
