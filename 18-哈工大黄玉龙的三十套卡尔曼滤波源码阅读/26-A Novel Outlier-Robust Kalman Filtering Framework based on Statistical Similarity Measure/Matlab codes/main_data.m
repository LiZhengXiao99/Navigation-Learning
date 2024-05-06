%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%% Model parameters
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
ts=200;

%% Outlier parameters
U1=100;          
U2=500;          
p1=0.95;         
p2=0.95;         

%%
xA=zeros(nx,ts+1,nxp);
zA=zeros(nz,ts,nxp);

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %% Initial true state
    x=[0;0;10;10];                        
    
    %% 
    xA(:,1,expt)=x;

    %% Simulate true state and measurement
    for t=1:ts

        %% Simulate heavy-tailed state noise
        test1=rand;
        if test1<=p1
            Q=Q0;
        else
            Q=U1*Q0;
        end
        SQ=utchol(Q);  
        vx=SQ*randn(nx,1);
        
        %% Simulate heavy-tailed measurement noise
        test2=rand;
        if test2<=p2
            R=R0;
        else
            R=U2*R0;
        end
        SR=utchol(R);
        vz=SR*randn(nz,1);

        %% Simulate state and measurement
        x=F*x+vx;
        z=H*x+vz;
        
        %% Save data
        xA(:,t+1,expt)=x;
        zA(:,t,expt)=z;
        
    end

end

%% 
XA=xA;
ZA=zA;

%% Save true state and measurement
save('data\imu.mat','XA','ZA');