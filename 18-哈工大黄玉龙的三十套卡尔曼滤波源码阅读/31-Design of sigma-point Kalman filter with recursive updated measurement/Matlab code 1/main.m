%%%%%%%%%%%%
%Y. L. Huang, Y. G. Zhang, N. Li, and L. Zhao, Design of sigma-point Kalman
%filter with recursive updated measurement. Circuits, Systems, and Signal
%Processing, vol. 35, no. 5, pp. 1767-1782, May 2016. 

%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock)); 
format long;

%%%%%%%%%%%%Simulation parameters
nxp=1000;                  %%%%%%%%The number of Montel Carlo
T=30;                      %%%%%%%%Simulation time
n=4;                       
G=[0.5 0;1 0;0 0.5;0 1];
N=10;                      %%%%%%%%The number of iteration  

%%%%%%%%%%%%Process noise covariance matrix
Q0=0.001^2*eye(2);
SQ=utchol(Q0);   
Q=G*Q0*G';       
%%%%%%%%%%%%Measurement noise variance
R=10*0.005^2;    
SR=sqrt(R);


for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%%%%%Initial condition
    x=[-0.05;0.001;0.7;-0.055]; 
    P=10*diag([0.1^2 0.005^2 0.1^2 0.01^2]); 
    Skk=utchol(P);  

    %%%%%%%%%%%%EKF with recursive update
    ru_xe=x+Skk*randn(n,1); 
    ru_Pe=P;

    %%%%%%%%%%%%The proposed CKF with recursive update
    ru_xc=ru_xe;
    ru_Pc=ru_Pe;

    %%%%%%%%%%%%Save data
    xA=x;
    ru_xeA=ru_xe;
    ru_xcA=ru_xc;

    for t=1:T
        
        %%%%%%%%%%%%
        x=ProssEq(x)+G*SQ*randn(2,1);
        z=MstEq(x)+SR*randn;
        
        %%%%%%%%%%%%Filtering
        [ru_xe,ru_Pe]=ruekf(ru_xe,ru_Pe,z,Q,R,N);

        [ru_xc,ru_Pc]=ruckf(ru_xc,ru_Pc,z,Q,R,N);
        
        %%%%%%%%%%%%Save data
        xA=[xA x];  
        ru_xeA=[ru_xeA ru_xe];
        ru_xcA=[ru_xcA ru_xc];
        
        %%%%%%%%%%%%Calculate MSE
        MSE_ruekf_pos(t,expt)=(xA(1,t)-ru_xeA(1,t))^2+(xA(3,t)-ru_xeA(3,t))^2;
        MSE_ruekf_vel(t,expt)=(xA(2,t)-ru_xeA(2,t))^2+(xA(4,t)-ru_xeA(4,t))^2;
     
        MSE_ruckf_pos(t,expt)=(xA(1,t)-ru_xcA(1,t))^2+(xA(3,t)-ru_xcA(3,t))^2;
        MSE_ruckf_vel(t,expt)=(xA(2,t)-ru_xcA(2,t))^2+(xA(4,t)-ru_xcA(4,t))^2;

    end
    
end

%%%%%%%%%%%%Calculate LMSE
MSE_ruekf_pos=log10(mean(MSE_ruekf_pos,2));
MSE_ruekf_vel=log10(mean(MSE_ruekf_vel,2));
    
MSE_ruckf_pos=log10(mean(MSE_ruckf_pos,2));
MSE_ruckf_vel=log10(mean(MSE_ruckf_vel,2));   
    
%%%%%%%%%%%%Figure
figure;
j = 1:T;
plot(j,MSE_ruekf_pos,'-*k',j,MSE_ruckf_pos,'-ok');
legend('Existing EKF with recursive update','The proposed method');
xlabel('Time(s)');
ylabel('LMSE_{pos}'); 

figure;
j = 1:T;
plot(j,MSE_ruekf_vel,'-*k',j,MSE_ruckf_vel,'-ok');
legend('Existing EKF with recursive update','The proposed method');
xlabel('Time(s)');
ylabel('LMSE_{vel}'); 
