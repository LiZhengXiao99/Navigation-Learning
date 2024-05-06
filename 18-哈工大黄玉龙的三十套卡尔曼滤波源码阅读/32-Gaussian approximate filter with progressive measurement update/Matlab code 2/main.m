%%%%%%%%%%%%
%Y. L. Huang, Y. G. Zhang, N. Li, and L. Zhao, Gaussian approximate filter
%with progressive measurement update. In 2015 IEEE 54th Annual Conference
%on Decision and Control (CDC), pp. 4344-4349, December 15-18, 2015. Osaka,
%Japan

%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock)); 
format long;

%%%%%%%%%%%%Simulation parameters
nxp=1000;    
T=30;       
n=4;         
G=[0.5 0;1 0;0 0.5;0 1];
N=30;        

%%%%%%%%%%%%Process noise covariance matrix
Q0=0.001^2*eye(2);
SQ=utchol(Q0);   
Q=G*Q0*G';       
%%%%%%%%%%%%Measurement noise variance
R=100*0.005^2;       
SR=sqrt(R);


for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%%%%%Initial condition
    x=[-0.05;0.001;0.7;-0.055]; 
    P=diag([0.1^2 0.005^2 0.1^2 0.01^2]); 
    Skk=utchol(P);  

    %%%%%%%%%%%%EKF with recursive update
    ru_xe=x+Skk*randn(n,1); 
    ru_Pe=P;
    
    %%%%%%%%%%%%CKF
    xc=ru_xe;
    Pc=ru_Pe;

    %%%%%%%%%%%%The proposed CKF with recursive update
    New_p_xc=ru_xe;
    New_p_Pc=ru_Pe;
    
    %%%%%%%%%%%%Save data
    xA=x;
    xcA=xc;
    ru_xeA=ru_xe;
    New_p_xcA=New_p_xc;

    for t=1:T
        
        %%%%%%%%%%%%
        x = ProssEq(x)+G*SQ*randn(2,1);
        z = MstEq(x)+SR*randn;
        
        %%%%%%%%%%%%Filtering
        [xc,Pc]=ckf(xc,Pc,z,Q,R);
        
        [ru_xe,ru_Pe]=ruekf(ru_xe,ru_Pe,z,Q,R,N);

        [New_p_xc,New_p_Pc]=New_pckf(New_p_xc,New_p_Pc,z,Q,R,N);
       
        %%%%%%%%%%%%Save data
        xA=[xA x];  
        xcA=[xcA xc];
        ru_xeA=[ru_xeA ru_xe];
        New_p_xcA=[New_p_xcA New_p_xc];
        
        %%%%%%%%%%%%Calculate MSE
        MSE_ckf_pos(t,expt)=(xA(1,t)-xcA(1,t))^2+(xA(3,t)-xcA(3,t))^2;
        MSE_ckf_vel(t,expt)=(xA(2,t)-xcA(2,t))^2+(xA(4,t)-xcA(4,t))^2;
       
        MSE_ruekf_pos(t,expt)=(xA(1,t)-ru_xeA(1,t))^2+(xA(3,t)-ru_xeA(3,t))^2;
        MSE_ruekf_vel(t,expt)=(xA(2,t)-ru_xeA(2,t))^2+(xA(4,t)-ru_xeA(4,t))^2;
      
        MSE_New_pckf_pos(t,expt)=(xA(1,t)-New_p_xcA(1,t))^2+(xA(3,t)-New_p_xcA(3,t))^2;
        MSE_New_pckf_vel(t,expt)=(xA(2,t)-New_p_xcA(2,t))^2+(xA(4,t)-New_p_xcA(4,t))^2;

    end
    
end

%%%%%%%%%%%%Calculate LMSE  
MSE_ckf_pos=log10(mean(MSE_ckf_pos,2));
MSE_ckf_vel=log10(mean(MSE_ckf_vel,2));

MSE_ruekf_pos=log10(mean(MSE_ruekf_pos,2));
MSE_ruekf_vel=log10(mean(MSE_ruekf_vel,2));
 
MSE_New_pckf_pos=log10(mean(MSE_New_pckf_pos,2));
MSE_New_pckf_vel=log10(mean(MSE_New_pckf_vel,2));

%%%%%%%%%%%%Figure
figure;
j = 1:T;
plot(j,MSE_ckf_pos,'-pk',j,MSE_ruekf_pos,'-*k',j,MSE_New_pckf_pos,'-ok');
legend('Standard CKF','Existing EKF with recursive update','The proposed method');
xlabel('Time(s)');
ylabel('LMSE_{pos}'); 

figure;
j = 1:T;
plot(j,MSE_ckf_vel,'-pk',j,MSE_ruekf_vel,'-*k',j,MSE_New_pckf_vel,'-ok');
legend('Standard CKF','Existing EKF with recursive update','The proposed method');
xlabel('Time(s)');
ylabel('LMSE_{vel}'); 
