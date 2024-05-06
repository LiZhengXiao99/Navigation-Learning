%%%%%
clear all;
close all;
clc;
format long;
randn('state',sum(100*clock));

%%%%%
nxp=1000;%%%Monte Carlo Simulation number
T=200;   %%%Simulation time
n=2;     %%%System dimension

%%%%%Parameter for ECKF
delta=sqrt(1);

%%%%%Parameter for high-degree ECKF
high_delta=0.8419;

%%%%%Search step size for adaptive ECKF and adaptive high-degree ECKF
step=0.01;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%Parameter setup
    Q=0.00001*[1 0;0 1];
    SQ=utchol(Q);
    R=0.05;     
    SR=sqrt(R);
    P=diag([400 25]);  
    x= [20 5]';

    %%%%CKF initial value
    xc=x+utchol(P)*randn(n,1);
    Pc=P;

    %%%%ECKF initial value
    e_xc=xc;
    e_Pc=P;

    %%%%High-degree ECKF initial value
    high_e_xc=xc;
    high_e_Pc=P;

    %%%%Adaptive ECKF initial value
    ae_xc=xc;
    ae_Pc=P;
    
    %%%%Adaptive High-degree ECKF initial value
    high_ae_xc=xc;
    high_ae_Pc=P;
         
    %%%%Save data
    xA=x;
    xcA=xc;
    e_xcA=e_xc;
    high_e_xcA=high_e_xc;
    ae_xcA=ae_xc;
    high_ae_xcA=high_ae_xc;

    for t=1:T
        
        x=ProssEq(x)+SQ*randn(n,1);  
        z=MstEq(x,t)+sqrt(R)*randn;

        %%%%Filtering
        [xc,Pc]=ckf(xc,Pc,z,Q,R,t);
        
        [e_xc,e_Pc]=eckf(e_xc,e_Pc,z,Q,R,delta,t);
        
        [high_e_xc,high_e_Pc]=high_eckf(high_e_xc,high_e_Pc,z,Q,R,high_delta,t);
        
        [ae_xc,ae_Pc,a_delta]=adaptive_eckf(ae_xc,ae_Pc,z,Q,R,step,t);
        
        [high_ae_xc,high_ae_Pc,a_high_delta]=adaptive_high_eckf(high_ae_xc,high_ae_Pc,z,Q,R,step,t);
        
        %%%%Save data
        xA=[xA x];
        xcA=[xcA xc];
        e_xcA=[e_xcA e_xc];
        high_e_xcA=[high_e_xcA high_e_xc];
        ae_xcA=[ae_xcA ae_xc];
        high_ae_xcA=[high_ae_xcA high_ae_xc];
      
        %%%%Calculate MSE
        MSE_ckf_Pos(1,t,expt)=(xA(1,t)-xcA(1,t))^2;
        MSE_ckf_Vel(1,t,expt)=(xA(2,t)-xcA(2,t))^2;

        MSE_e_ckf_Pos(1,t,expt)=(xA(1,t)-e_xcA(1,t))^2;
        MSE_e_ckf_Vel(1,t,expt)=(xA(2,t)-e_xcA(2,t))^2;
        
        MSE_high_e_ckf_Pos(1,t,expt)=(xA(1,t)-high_e_xcA(1,t))^2;
        MSE_high_e_ckf_Vel(1,t,expt)=(xA(2,t)-high_e_xcA(2,t))^2;

        MSE_ae_ckf_Pos(1,t,expt)=(xA(1,t)-ae_xcA(1,t))^2;
        MSE_ae_ckf_Vel(1,t,expt)=(xA(2,t)-ae_xcA(2,t))^2;
        
        MSE_high_ae_ckf_Pos(1,t,expt)=(xA(1,t)-high_ae_xcA(1,t))^2;
        MSE_high_ae_ckf_Vel(1,t,expt)=(xA(2,t)-high_ae_xcA(2,t))^2;

    end
    
end

%%%%Calculate average MSE
MSE_ckf_Pos=mean(MSE_ckf_Pos,3);
MSE_ckf_Vel=mean(MSE_ckf_Vel,3);
     
MSE_e_ckf_Pos=mean(MSE_e_ckf_Pos,3);
MSE_e_ckf_Vel=mean(MSE_e_ckf_Vel,3);
     
MSE_high_e_ckf_Pos=mean(MSE_high_e_ckf_Pos,3);
MSE_high_e_ckf_Vel=mean(MSE_high_e_ckf_Vel,3);

MSE_ae_ckf_Pos=mean(MSE_ae_ckf_Pos,3);
MSE_ae_ckf_Vel=mean(MSE_ae_ckf_Vel,3);
     
MSE_high_ae_ckf_Pos=mean(MSE_high_ae_ckf_Pos,3);
MSE_high_ae_ckf_Vel=mean(MSE_high_ae_ckf_Vel,3);

%%%%MSE curves
figure;
j = 1:T;
plot(j,MSE_ckf_Pos(1,:),'-*k',j,MSE_e_ckf_Pos(1,:),':k',j,MSE_high_e_ckf_Pos,'-k',j,MSE_ae_ckf_Pos(1,:),'-g',j,MSE_high_ae_ckf_Pos,'--k','linewidth',2.0);
legend('3rd-CKF','3rd-ECKF (\delta=1)','The proposed 5th-ECKF (\delta=0.8419)','The proposed 3rd-AECKF','The proposed 5th-AECKF');
xlabel('Time(s)');
ylabel('MSE of x_1'); 

figure;
j = 1:T;
plot(j,MSE_ckf_Vel(1,:),'-*k',j,MSE_e_ckf_Vel(1,:),':k',j,MSE_high_e_ckf_Vel,'-k',j,MSE_ae_ckf_Vel(1,:),'-g',j,MSE_high_ae_ckf_Vel,'--k','linewidth',2.0);
legend('3rd-CKF','3rd-ECKF (\delta=1)','The proposed 5th-ECKF (\delta=0.8419)','The proposed 3rd-AECKF','The proposed 5th-AECKF');
xlabel('Time(s)');
ylabel('MSE of x_2'); 
