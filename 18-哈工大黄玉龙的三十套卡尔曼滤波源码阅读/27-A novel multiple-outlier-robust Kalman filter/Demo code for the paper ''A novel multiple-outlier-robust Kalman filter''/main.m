profile on ;
clear all;
close all;
clc;
randn('state',sum(100*clock));    
format long;

%%%%%model parameters%%%%%%%%%%%%%%%%
nxp=10;
nx=4;
nz=2;
T=1;        
q=1;        
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=1000;

%%%%settings
N=50;        %%%%%%number for VB iterations
gama=1.345;  %%%%%%tuning paramaeter for HKF
sigma=15;    %%%%%%kernel size for MCKF
v=5;         %%%%%%dof for Student's t 

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%initial values%%%%%%%%%%%
    x=[0;0;10;10];                       
    P=diag([10000 10000 100 100]);       
    Skk=utchol(P);                        

    %%%%     (Kalman filter)
    xf=x+Skk*randn(nx,1);                
    Pf=P;
    
    %%%%ME          (M-estimation)
    xme=xf;
    Pme=Pf;

    %%%%MCCKF      Chen Badong
    xmcc=xf;
    Pmcc=Pf;
    
    %%%%RSTKF
    xiv=xf;
    Piv=Pf;

    %%%%SSMKF_1     flag=1 
    xssm_11=xf;
    Pssm_11=Pf;
    
    %%%%SSMKF_2     flag=2
    xssm_12=xf;
    Pssm_12=Pf;
    
    %%%%SSMKF_3     flag=3
    xssm_13=xf;
    Pssm_13=Pf;
    
    %%%%SSMKF_2     flag=1 
    xssm_21=xf;
    Pssm_21=Pf;
    
    %%%%SSMKF_2     flag=2 
    xssm_22=xf;
    Pssm_22=Pf;
    
    %%%%SSMKF_2     flag=3
    xssm_23=xf;
    Pssm_23=Pf;

    %%%%data saving
    xA=x;
    xfA=xf;
    xmeA=xme;
    xmccA=xmcc;
    xivA=xiv;
    xssm_13A=xssm_13;
    xssm_23A=xssm_23;
    %%%paramters
    lamda1A=[];
    lamda2A=[];
    Pu_xA=[];
    Pu_zA=[];
    
    for t=1:ts
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%control parameters for outliers
        alfa_ax=6; 
        alfa_ay=8;  
        alfa_x=10; 
        alfa_y=6;
        U0=50*alfa_ax;
        U1=50*alfa_ay;    
        U2=50*alfa_x;    
        U3=50*alfa_y;    
        p1=0.97;       
        p2=0.97;        
        p3=0.90;       
        
        %%%%nonminal covariances for KF
        D_Q=p1*Q0+(1-p1)*U1*Q0;
        D_R=p2*R0+(1-p2)*U2*R0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%stage 1
        if t>=1 && t<=ts/2
            %%%same outliers stage 
            test1=rand;
            if test1<=p1
                Q=Q0;
            else
                Q=U0*Q0;
            end
            %%%
            test2=rand;
            if test2<=p2
                R=R0;
            else
                R=U2*R0;
            end
        end
        
        %%%stage 2
        if t>=ts/2+1 && t<=ts
            %%%multiple outliers stage
            testw1=rand;
            testw2=rand;
            %%%%
            if testw1<=p1
                Q(1,1)=Q0(1,1);
                Q(1,3)=Q0(1,3);
                Q(3,3)=Q0(3,3);
                Q(3,1)=Q0(3,1);
            else
                Q(1,1)=U0*Q0(1,1);
                Q(1,3)=U0*Q0(1,3);
                Q(3,3)=U0*Q0(3,3);
                Q(3,1)=U0*Q0(3,1);
            end
            %%%%
            if testw2<=p3
                Q(2,2)=Q0(2,2);
                Q(2,4)=Q0(2,4);
                Q(4,2)=Q0(4,2);
                Q(4,4)=Q0(4,4);
            else
                Q(2,2)=U1*Q0(2,2);
                Q(2,4)=U1*Q0(2,4);
                Q(4,2)=U1*Q0(4,2);
                Q(4,4)=U1*Q0(4,4);
            end

            testv1=rand;
            testv2=rand;   
            if testv1<=p2
                R(1,1)=R0(1,1);
            else
                R(1,1)=U2*R0(1,1);
            end
            %%%%
            if testv2<=p3
                R(2,2)=R0(2,2);
            else
                R(2,2)=U3*R0(2,2);
            end
        end
       
        %%%%
        SQ=utchol(Q);  
        SR=utchol(R);
        wx=SQ*randn(nx,1);
        vz=SR*randn(nz,1);
        
        %%%%state-space model
        x=F*x+wx;
        z=H*x+vz;
        
        %%%%filters
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);

        %%%%existing SSMKF
        [xssm_13,Pssm_13,lamda1,lamda2]=ssmkf_1(xssm_13,Pssm_13,F,H,z,Q0,R0,sigma,v,N,3); 
        
        %%%%proposed filter
        [xssm_23,Pssm_23,Pu_x,Pu_z]=ssmkf_2(xssm_23,Pssm_23,F,H,z,Q0,R0,sigma,v,N,3);

        %%%%Êı¾İ´æ´¢
        xA=[xA x];
        xfA=[xfA xf];
        xssm_13A=[xssm_13A xssm_13];
        xssm_23A=[xssm_23A xssm_23];


        %%%%MSE
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;

        mse_ssmkf_13_1(1,t,expt)=(xA(1,t+1)-xssm_13A(1,t+1))^2+(xA(2,t+1)-xssm_13A(2,t+1))^2;
        mse_ssmkf_13_2(1,t,expt)=(xA(3,t+1)-xssm_13A(3,t+1))^2+(xA(4,t+1)-xssm_13A(4,t+1))^2;

        mse_ssmkf_23_1(1,t,expt)=(xA(1,t+1)-xssm_23A(1,t+1))^2+(xA(2,t+1)-xssm_23A(2,t+1))^2;
        mse_ssmkf_23_2(1,t,expt)=(xA(3,t+1)-xssm_23A(3,t+1))^2+(xA(4,t+1)-xssm_23A(4,t+1))^2;
        
        %%%%MSE for middle parameters
        mse_lamda1(1,t,expt)=lamda1;
        mse_lamda2(1,t,expt)=lamda2;
        mse_Pu_x1(1,t,expt)=Pu_x(1,1);
        mse_Pu_x2(1,t,expt)=Pu_x(2,2);
        mse_Pu_x3(1,t,expt)=Pu_x(3,3);
        mse_Pu_x4(1,t,expt)=Pu_x(4,4);
        mse_Pu_z1(1,t,expt)=Pu_z(1,1);
        mse_Pu_z2(1,t,expt)=Pu_z(2,2);
        
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSEs
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_ssmkf_13_1=sqrt(mean(mse_ssmkf_13_1,3));
rmse_ssmkf_13_2=sqrt(mean(mse_ssmkf_13_2,3));

rmse_ssmkf_23_1=sqrt(mean(mse_ssmkf_23_1,3));
rmse_ssmkf_23_2=sqrt(mean(mse_ssmkf_23_2,3));

%%%%%%%%%RMSEs for middle  parameters
rmse_lamda1=mean(mse_lamda1,3);
rmse_lamda2=mean(mse_lamda2,3);
rmse_Pu_x1=mean(mse_Pu_x1,3);
rmse_Pu_x2=mean(mse_Pu_x2,3);
rmse_Pu_x3=mean(mse_Pu_x3,3);
rmse_Pu_x4=mean(mse_Pu_x4,3);
rmse_Pu_z1=mean(mse_Pu_z1,3);
rmse_Pu_z2=mean(mse_Pu_z2,3);

  
%%%%%%%%%ARMSEs
initalPoint=600;
armse_kf_1=mean(rmse_kf_1(initalPoint:end))
armse_kf_2=mean(rmse_kf_2(initalPoint:end))

armse_ssmkf_13_1=mean(rmse_ssmkf_13_1(initalPoint:end))
armse_ssmkf_13_2=mean(rmse_ssmkf_13_2(initalPoint:end))

armse_ssmkf_23_1=mean(rmse_ssmkf_23_1(initalPoint:end))
armse_ssmkf_23_2=mean(rmse_ssmkf_23_2(initalPoint:end))

%%%%%%%%%Smooth the SRMSEs
stepsize=50;
srmse_kf_1=smooth(rmse_kf_1,stepsize);
srmse_kf_2=smooth(rmse_kf_2,stepsize);

srmse_ssmkf_13_1=smooth(rmse_ssmkf_13_1,stepsize);
srmse_ssmkf_13_2=smooth(rmse_ssmkf_13_2,stepsize);

srmse_ssmkf_23_1=smooth(rmse_ssmkf_23_1,stepsize);
srmse_ssmkf_23_2=smooth(rmse_ssmkf_23_2,stepsize);

%%%%plot parameters
figure;
j=1:ts;
plot(j*T,rmse_lamda1,'-k',j*T,rmse_lamda2,'-b','linewidth',2.5);
xlabel('Time (s)');
ylabel('Scalar factors for SSMKF');
legend('\xi','\lambda');

figure;
j=1:ts;
plot(j*T,rmse_lamda1,'-k',j*T,rmse_Pu_x1,'-c',j*T,rmse_Pu_x2,'-b',j*T,rmse_Pu_x3,'-m',j*T,rmse_Pu_x4,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('Matrix \Psi_x for MORKF');
legend('\xi of SSMKF','\Psi_x(1,1) of MORKF','\Psi_x(2,2) of MORKF','\Psi_x(3,3) of MORKF','\Psi_x(4,4) of MORKF');

figure;
j=1:ts;
plot(j*T,rmse_lamda2,'-k',j*T,rmse_Pu_z1,'-b',j*T,rmse_Pu_z2,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('Matrix \Psi_z for MORKF');
legend('\lambda of SSMKF','\Psi_z(1,1) of MORKF','\Psi_z(2,2) of MORKF');

%%%%plot
figure;
j=1:ts;
plot(j*T,srmse_kf_1,'-k','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_13_1,'-c',j*T,srmse_ssmkf_23_1,'-r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{pos} (m)');
legend('KFTCM','SSMKF-sqrt','Proposed MORKF-sqrt');

figure;
j=1:ts;
plot(j*T,srmse_kf_2,'-k','linewidth',2.5);
hold on;
plot(j*T,srmse_ssmkf_13_2,'-c',j*T,srmse_ssmkf_23_2,'-r','linewidth',2.5)
xlabel('Time (s)');
ylabel('SRMSE_{vel} (m/s)');
legend('KFTCM','SSMKF-sqrt','Proposed MORKF-sqrt');

profile viewer

% save('data\,matrix1.mat','rmse_lamda1','rmse_Pu_x1','rmse_Pu_x2','rmse_Pu_x3','rmse_Pu_x4');
% save('data\,matrix2.mat','rmse_lamda2','rmse_Pu_z1','rmse_Pu_z2');
% save('data\,SRMSEpos.mat','srmse_kf_1','srmse_mekf_1','srmse_mcckf_1','srmse_ssmkf_13_1','srmse_ssmkf_23_1');
% save('data\,SRMSEvel.mat','srmse_kf_2','srmse_mekf_2','srmse_mcckf_2','srmse_ssmkf_13_2','srmse_ssmkf_23_2');
% save('data\,RMSEpos.mat','rmse_kf_1','rmse_mekf_1','rmse_mcckf_1','rmse_ssmkf_13_1','rmse_ssmkf_23_1');
% save('data\,RMSEpos.mat','rmse_kf_2','rmse_mekf_2','rmse_mcckf_2','rmse_ssmkf_13_2','rmse_ssmkf_23_2');

