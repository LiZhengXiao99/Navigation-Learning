%%%%%This program generates "swrkf_data.mat".
% Output:
% XA True value of the state trajectroy
% ZA Measurements
% qA Whether there is a state outlier at the corresponding time stamp and Monte Carlo time
% rA Whether there is a measurement outlier at the corresponding time stamp and Monte Carlo time
% p1 Probability of state outliers
% p2 Probability of measurement outliers
% U1 Covariance magnification of the state outliers
% U2 Covariance magnification of the measurement outliers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));
format long;

%%%%%Parameters of the model%%%%%%%%%%%%%%%%
nxp=10;
nx=4;
nz=2;
T=1;        
q=0.5; 
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=5000;
U1=100;      
U2=100;      
p1=0.9;     
p2=0.9;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%Initial state%%%%%%%%%%%
    x=[0;0;10;10];                                      %%%True state
    P=diag([10000 10000 100 100]);        %%%Initial state covariance
    
    for t=1:ts
        
        %%%%Simulate true state and measurement
        test1=rand;
        test2=rand;
        
        if test1<=p1
            Q=Q0;
            q=1;
        else
            Q=U1*Q0;
            q=100;
        end

        if test2<=p2
            R=R0;
            r=1;
        else
            R=U2*R0;
            r=100;
        end
        
        %%%%Calculate square-root matrix
        SQ=utchol(Q);    
        SR=utchol(R);    
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);
        %%%%Data saving
        XA(:,t,expt)=x;
        ZA(:,t,expt)=z;
        qA(t,expt)=q;
        rA(t,expt)=r;
    end

end
save swrkf_data.mat XA ZA qA rA p1 p2 U1 U2