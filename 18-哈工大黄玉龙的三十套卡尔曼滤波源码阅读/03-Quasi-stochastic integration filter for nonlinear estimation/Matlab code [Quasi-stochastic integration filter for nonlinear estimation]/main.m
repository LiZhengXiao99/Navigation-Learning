%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%
nxp=1000;
T=100;
nx=2;
Nmax=5;    %%%%iteration numbers

%%%%
Q=0.1*[1 0.5;0.5 1];
R=0.025;
SQ=utchol(Q);

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 

    %%%%
    x=[20 5]';
    P=diag([0.1 0.1]);
    SP=utchol(P);
    
    %%%%CKF    
    xc=x+SP*randn(nx,1);
    Pc=P;
    
    %%%%3rd-QSIF
    SR_xc=xc;
    SR_Pc=P;
    
    %%%%5th-QSIF
    SR_high_xc=xc;
    SR_high_Pc=P;

    %%%%Save data
    xA=x;
    xcA=xc;
    SR_xcA=SR_xc;
    SR_high_xcA=SR_high_xc;

    for t=1:T
        
        x=ProssEq(x)+SQ*randn(nx,1);  
        z=MstEq(x,t)+sqrt(R)*randn;
        
        %%%%Filtering
        [xc,Pc]=ckf(xc,Pc,z,Q,R,t);
        
        [SR_xc,SR_Pc] = SR_ckf(SR_xc,SR_Pc,z,Q,R,t,Nmax);
        
        [SR_high_xc,SR_high_Pc] = SR_high_ckf(SR_high_xc,SR_high_Pc,z,Q,R,t,Nmax);

        %%%%Save data
        xA=[xA x];
        
        xcA=[xcA xc];
        
        SR_xcA=[SR_xcA SR_xc];
        
        SR_high_xcA=[SR_high_xcA SR_high_xc];
        
        %%%%Calculate MSE
        MSE_ckf_Pos(1,t,expt)=(xA(1,t)-xcA(1,t))^2;
        MSE_ckf_Vel(1,t,expt)=(xA(2,t)-xcA(2,t))^2;
        
        MSE_SR_ckf_Pos(1,t,expt)=(xA(1,t)-SR_xcA(1,t))^2;
        MSE_SR_ckf_Vel(1,t,expt)=(xA(2,t)-SR_xcA(2,t))^2;
        
        MSE_SR_high_ckf_Pos(1,t,expt)=(xA(1,t)-SR_high_xcA(1,t))^2;
        MSE_SR_high_ckf_Vel(1,t,expt)=(xA(2,t)-SR_high_xcA(2,t))^2;
       
    end
    
end
    
%%%%Calculate average MSE
MSE_ckf_Pos=mean(MSE_ckf_Pos,3);
MSE_ckf_Vel=mean(MSE_ckf_Vel,3);   

MSE_SR_ckf_Pos=mean(MSE_SR_ckf_Pos,3);
MSE_SR_ckf_Vel=mean(MSE_SR_ckf_Vel,3);

MSE_SR_high_ckf_Pos=mean(MSE_SR_high_ckf_Pos,3);
MSE_SR_high_ckf_Vel=mean(MSE_SR_high_ckf_Vel,3);
     
%%%%MSE curves    
figure;
j = 1:T;
plot(j,MSE_ckf_Pos(1,:),'-k',j,MSE_SR_ckf_Pos(1,:),'-b',j,MSE_SR_high_ckf_Pos(1,:),'-r','LineWidth',2.5);
legend('3rd-CKF','The proposed 3rd-QSIF','The proposed 5th-QSIF');
xlabel('time(s)');
ylabel('MSE of x_1');
hold on;

figure;
j = 1:T;
plot(j,MSE_ckf_Vel(1,:),'-k',j,MSE_SR_ckf_Vel(1,:),'-b',j,MSE_SR_high_ckf_Vel(1,:),'-r','LineWidth',2.5);
legend('3rd-CKF','The proposed 3rd-QSIF','The proposed 5th-QSIF');
xlabel('time(s)');
ylabel('MSE of x_2');
hold on;
