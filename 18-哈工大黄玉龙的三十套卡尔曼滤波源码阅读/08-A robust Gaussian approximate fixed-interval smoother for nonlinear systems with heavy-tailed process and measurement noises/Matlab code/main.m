%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Y. L. Huang, Y. G. Zhang, N. Li, and J. Chambers, A robust Gaussian
%approximate fixed-interval smoother for nonlinear systems with
%heavy-tailed process and measurement noises, IEEE Signal Processing
%Letters, vol. 23, no. 4, pp. 468-472, Apr. 2016. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
profile on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%%%%%%%%%%%%%%%%%%
raddeg=pi/180;                     
nx=4;                              
nz=2;                              
nxp=1000;                          
ts=200;   
T=0.5;
F=[eye(2) T*eye(2);zeros(2) eye(2)];

%%%%%%%%%%%%%%%%%%%%%
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)];
R0=diag([100 (4e-3*raddeg)^2]);

%%%%%%%%%%%%%%%%%%%%%
p1=0.80;
p2=0.80;

%%%%%%%%%%%%%%%%%%%%%
N=10;           

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%%%%%%%%%%%%%Initialization
    x=[10000;1000;300;-40];               
    P=diag([100 100 100 100]);            
    Skk=utchol(P);                        

    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-QR\omega\nu (the proposed robust CKS with estimated Q, R, w and v)
    xi=x+Skk*randn(nx,1);                 
    Pi=P;
    
    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-fixed (the proposed robust CKS with fixed noise parameters)
    xfi=xi;                
    Pfi=Pi;
    
    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-QR (the proposed robust CKS with estimated Q and R and fixed w and v)
    xfdi=xi;                
    Pfdi=Pi;
    
    %%%%%%%%%%%%%%%%%%%%%The proposed CKS-\omega\nu (the proposed robust CKS with estimated w and v and fixed Q and R) 
    xfci=xi;                
    Pfci=Pi;
    
    %%%%%%%%%%%%%%%%%%%%%Standard CKS
    xs=xi;
    Ps=Pi;

    %%%%%%%%%%%%%%%%%%%%%
    xA=x;
    zA=[];

    %%%%%%%%%%%%%%%%%%%%%
    for t=1:ts
        
        test1=rand;
        test2=rand;
        
        if test1<=p1
            Q=Q0;
        else
            Q=1000*Q0;
        end
        
        if test2<=p2
            R=R0;
        else
            R=100*R0;
        end

        %%%%%%%%%%%%%%%%%%%%%
        SQ=utchol(Q);    
        SR=utchol(R);
        
        %%%%%%%%%%%%%%%%%%%%%
        x=F*x+SQ*randn(nx,1);
        z=MstEq(x)+SR*randn(nz,1);

        xA=[xA x];  
        zA=[zA z];

    end

    %%%%%%%%%%%%%%%%%%%%%
    c0=5;
    d0=1;
    t0=nx+2;
    T0=Q0;
    a0=5;
    b0=1;
    u0=nz+2;
    U0=R0;

    %%%%%%%%%%%%%%%%%%%%%
    [ixsBA,iPsBA]=icks(xi,Pi,F,zA,ts,c0,d0,t0,T0,a0,b0,u0,U0,N);      %%%%%The proposed CKS-QR\omega\nu
    
    [fixsBA,fiPsBA]=ficks(xfi,Pfi,F,zA,ts,Q0,R0,N);                   %%%%%The proposed CKS-fixed
    
    [fdixsBA,fdiPsBA]=fdicks(xfdi,Pfdi,F,zA,ts,t0,T0,u0,U0,N);        %%%%%The proposed CKS-QR
    
    [fcixsBA,fciPsBA]=fcicks(xfci,Pfci,F,zA,ts,c0,d0,Q0,a0,b0,R0,N);  %%%%%The proposed CKS-\omega\nu
    
    [sxsBA,sPsBA]=scks(xs,Ps,F,zA,ts,Q0,R0);                          %%%%%Standard CKS
 
    %%%%%%%%%%%%%%%%%%%%%
    mse_icks_1(expt,:)=(xA(1,:)-ixsBA(1,:)).^2+(xA(2,:)-ixsBA(2,:)).^2;
    mse_icks_2(expt,:)=(xA(3,:)-ixsBA(3,:)).^2+(xA(4,:)-ixsBA(4,:)).^2;
    
    mse_ficks_1(expt,:)=(xA(1,:)-fixsBA(1,:)).^2+(xA(2,:)-fixsBA(2,:)).^2;
    mse_ficks_2(expt,:)=(xA(3,:)-fixsBA(3,:)).^2+(xA(4,:)-fixsBA(4,:)).^2;
    
    mse_fdicks_1(expt,:)=(xA(1,:)-fdixsBA(1,:)).^2+(xA(2,:)-fdixsBA(2,:)).^2;
    mse_fdicks_2(expt,:)=(xA(3,:)-fdixsBA(3,:)).^2+(xA(4,:)-fdixsBA(4,:)).^2;
    
    mse_fcicks_1(expt,:)=(xA(1,:)-fcixsBA(1,:)).^2+(xA(2,:)-fcixsBA(2,:)).^2;
    mse_fcicks_2(expt,:)=(xA(3,:)-fcixsBA(3,:)).^2+(xA(4,:)-fcixsBA(4,:)).^2;

    mse_scks_1(expt,:)=(xA(1,:)-sxsBA(1,:)).^2+(xA(2,:)-sxsBA(2,:)).^2;
    mse_scks_2(expt,:)=(xA(3,:)-sxsBA(3,:)).^2+(xA(4,:)-sxsBA(4,:)).^2;
 
end
        
%%%%%%%%%%%%%%%%%%%%%Calculate RMSE
rmse_icks_1=sqrt(mean(mse_icks_1,1));
rmse_icks_2=sqrt(mean(mse_icks_2,1));
        
rmse_ficks_1=sqrt(mean(mse_ficks_1,1));
rmse_ficks_2=sqrt(mean(mse_ficks_2,1));
        
rmse_fdicks_1=sqrt(mean(mse_fdicks_1,1));
rmse_fdicks_2=sqrt(mean(mse_fdicks_2,1));
        
rmse_fcicks_1=sqrt(mean(mse_fcicks_1,1));
rmse_fcicks_2=sqrt(mean(mse_fcicks_2,1));

rmse_scks_1=sqrt(mean(mse_scks_1,1));
rmse_scks_2=sqrt(mean(mse_scks_2,1));

%%%%%%%%%%%%%%%%%%%%%%%
figure;
j=0:ts;
plot(j*T,rmse_scks_1,'--k',j*T,rmse_ficks_1,'-r',j*T,rmse_fdicks_1,'-g',j*T,rmse_fcicks_1,'-c',j*T,rmse_icks_1,'-k','linewidth',2.5);
ylabel('RMSE_{pos} (m)');
xlabel('Time (s)');
legend('Standard CKS','The proposed CKS-fixed','The proposed CKS-QR','The proposed CKS-\omega\nu','The proposed CKS-QR\omega\nu');

figure;
j=0:ts;
plot(j*T,rmse_scks_2,'--k',j*T,rmse_ficks_2,'-r',j*T,rmse_fdicks_2,'-g',j*T,rmse_fcicks_2,'-c',j*T,rmse_icks_2,'-k','linewidth',2.5);
ylabel('RMSE_{vel} (m/s)');
xlabel('Time (s)');
legend('Standard CKS','The proposed CKS-fixed','The proposed CKS-QR','The proposed CKS-\omega\nu','The proposed CKS-QR\omega\nu');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
profile viewer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%