%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A simulation demo for smoother in the paper
%
%[1] Mingming Bai, Yulong Huang, et al. ¡°A robust fixed-interval smoother for nonlinear systems with non-stationary 
%heavy-tailed state and measurement noises¡±. Signal Processing, 180(2021):107898.
%
%If you use our code in your publication, please cite our paper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

profile on  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%%%%%%%%%%%%%%%%%%
nxp=10;        
nx=1;                              
nz=1;                                         
F=0.5;
H=1;
Q0=100;
R0=100;
ts=400;

%%%%%%%%%%%%%%%%%%%%%
g=0.85;  
e=0.85;  
U1=100;     
U2=100; 
pStrongHt=0.7;   
pSlightHt=0.9;

%%%%%%%%%%%%%%%%%%%%%
N=30;    

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
        
    %%%%%Initial values
    x=100;                          
    P=100;        
    Skk=utchol(P);   
    
    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-QR\omega\nu (the proposed robust CKS with estimated Q, R, w and v)
    xi=x+Skk*randn(nx,1);                 
    Pi=P;
    
    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-fixed (the proposed robust CKS with fixed noise parameters)
    xfi=xi;                
    Pfi=Pi;

    %%%%%%%%%%%%%%%%%%%%%Standard CKS
    xs=xi;
    Ps=Pi;

    %%%%%%%%%%%%%%%%%%%%%
    xA=x;
    zA=[];
    QA=[];
    RA=[];
    
    %%%%%%%%%%%%%%%%%%%%%
    for t=1:ts
        
        %%%%%%%Simulate non-stationary noise
        %%%%%%%Gaussian noise
        if t<=100
            p1=1;
            p2=1;
        end
        %%%%%%%Slightly heavy-tailed noise
        if (t>100)&&(t<=200)
             p1=pStrongHt;
             p2=pStrongHt;
        end
        %%%%%%%Moderately heavy-tailed noise
        if (t>200)&&(t<=300)
             p1=pSlightHt;
             p2=pSlightHt;
        end
        %%%%%%%Gaussian noise
        if t>300
            p1=1;
            p2=1;
        end
        
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

        %%%%%%%%%%%%%%%%%%%%%
        SQ=utchol(Q);    
        SR=utchol(R);
        
        %%%%Simulate true state and measurement
        x=ProssEq(x)+SQ*randn(nx,1);
        z=MstEq(x)+SR*randn(nz,1);  
        
        xA=[xA x];  
        zA=[zA z];
        QA=[QA Q];
        RA=[RA R];
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%
    taoQ=5;
    taoR=5;
    c0=5;
    d0=1;
    t0=nx+1+taoQ;
    T0=Q0;
    a0=5;
    b0=1;
    u0=nz+1+taoR;
    U0=R0;

    %%%%%%%%%%%%%%%%%%%%%
    %%%%%The proposed  
    [fixsBA,fiPsBA,E_tao,E_pi]=ficks(xfi,Pfi,F,H,zA,ts,Q0,R0,e,g,N);
    
    %%%%%The CKS-QR\omega\nu
    [ixsBA,iPsBA,E_Q,E_R,E_vw,E_vv]=icks(xi,Pi,F,H,zA,ts,c0,d0,t0,T0,a0,b0,u0,U0,N);    

    %%%%%Standard CKS
    [sxsBA,sPsBA]=scks(xs,Ps,F,zA,ts,QA,RA);    

    %%%%%%%%%%%%%%%%%%%%%
    mse_icks_1(expt,:)=(xA(1,:)-ixsBA(1,:)).^2;
    mse_ficks_1(expt,:)=(xA(1,:)-fixsBA(1,:)).^2;
    mse_scks_1(expt,:)=(xA(1,:)-sxsBA(1,:)).^2;
 
end
        
%%%%%%%%%%%%%%%%%%%%%Calculate RMSE
rmse_icks_1=sqrt(mean(mse_icks_1,1));
rmse_ficks_1=sqrt(mean(mse_ficks_1,1));       
rmse_scks_1=sqrt(mean(mse_scks_1,1));

%%%%%%%%%%%%%%%%%%%%%%%
figure;
j=1:ts;
T=1;
plot(j*T,rmse_icks_1(2:end),'-b',...
    j*T,rmse_scks_1(2:end),'-k',j*T,rmse_ficks_1(2:end),'-r','linewidth',2.5);
ylabel('RMSE_{pos} (m)');
xlabel('Time (s)');
legend('RGAS','TNCMCKS','Proposed smoother');

srmse_icks_1=smooth(rmse_icks_1,10);
srmse_scks_1=smooth(rmse_scks_1,10);
srmse_ficks_1=smooth(rmse_ficks_1,10);

%%%%%%%%%%%%%%%%%%%%%%%
figure;
j=1:ts;
T=1;
plot(j*T,srmse_icks_1(2:end),'-b',...
    j*T,srmse_scks_1(2:end),'-k',j*T,srmse_ficks_1(2:end),'-r','linewidth',2.5);
ylabel('RMSE_{pos} (m)');
xlabel('Time (s)');
legend('RGAS','TNCMCKS','Proposed smoother');

