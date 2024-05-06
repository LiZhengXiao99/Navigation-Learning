%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%Model parameters
raddeg=pi/180;                     
nx=5;                              
nz=2;                              
nxp=100;                           
ts=100;                            
T=1;
Nk=ts/T;                           
M=[T^3/3 T^2/2;T^2/2 T];

%%%%CKF-2009 simulation parameters
q1=0.1;
q2=1.75e-4;
Cr=10;
Co=sqrt(10)*1e-3;

%%%%Noise covariance matrices
Q=[q1*M zeros(2,2) zeros(2,1);zeros(2,2) q1*M zeros(2,1);zeros(1,2) zeros(1,2) q2*T]; 
R0=diag([Cr^2 Co^2]);
p=0.70;              %%%%%The probability of normal measurement

%%%%ORCKF dof parameters
f_v_1=1.0;
f_v_2=2.0;
f_v_3=3.0;
f_v_4=4.0;
f_v_5=5.0;
N=5;                  %%%%The number of VB iteration

for expt = 1:nxp
    
    %%%%%
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%Initialization
    x=[1000;300;1000;0;-3*raddeg];        
    P=diag([100 10 100 10 1e-4]);         
    Skk=utchol(P);                        
    
    %%%%CKF
    xc=x+Skk*randn(nx,1);                 
    Pc=P;
    
    %%%%ORCKF_1
    or_xc_1=xc;
    or_Pc_1=Pc;
    
    %%%%ORCKF_2
    or_xc_2=xc;
    or_Pc_2=Pc;
    
    %%%%ORCKF_3
    or_xc_3=xc;
    or_Pc_3=Pc;
    
    %%%%ORCKF_4
    or_xc_4=xc;
    or_Pc_4=Pc;
    
    %%%%ORCKF_5
    or_xc_5=xc;
    or_Pc_5=Pc;
    
    %%%%Adaptive ORCKF (AORCKF)
    aor_xc=xc;
    aor_Pc=Pc;
    
    %%%%AORCKF parameters
    a=5;
    b=1;
    u=nz+2;
    U=R0;
    rou=1-exp(-5);

    %%%%Save data
    xA=x;
    or_xc_1A=or_xc_1;
    or_xc_2A=or_xc_2;
    or_xc_3A=or_xc_3;
    or_xc_4A=or_xc_4;
    or_xc_5A=or_xc_5;
    aor_xcA=aor_xc;

    for t=1:Nk

        %%%%
        test=rand;
        
        %%%%Simulate measurement outliers
        if test<=p
            R=R0;
        else
            R=100*R0;
        end
        
        %%%%
        SR=utchol(R);    
        SQ=utchol(Q);    
        
        %%%%True state and measurement
        x=ProssEq(x)+SQ*randn(nx,1);
        z=MstEq(x)+SR*randn(nz,1);
        
        %%%%Existing ORCKF with different dof parameters
        [or_xc_1,or_Pc_1]=orckf(or_xc_1,or_Pc_1,z,Q,R0,f_v_1,N);
        [or_xc_2,or_Pc_2]=orckf(or_xc_2,or_Pc_2,z,Q,R0,f_v_2,N);
        [or_xc_3,or_Pc_3]=orckf(or_xc_3,or_Pc_3,z,Q,R0,f_v_3,N);
        [or_xc_4,or_Pc_4]=orckf(or_xc_4,or_Pc_4,z,Q,R0,f_v_4,N);
        [or_xc_5,or_Pc_5]=orckf(or_xc_5,or_Pc_5,z,Q,R0,f_v_5,N);
        
        %%%%The proposed AORCKF
        [aor_xc,aor_Pc,a,b,u,U]=aorckf(aor_xc,aor_Pc,z,Q,a,b,u,U,rou,N);

        %%%%Save data
        xA=[xA x];  
        or_xc_1A=[or_xc_1A or_xc_1];
        or_xc_2A=[or_xc_2A or_xc_2];
        or_xc_3A=[or_xc_3A or_xc_3];
        or_xc_4A=[or_xc_4A or_xc_4];
        or_xc_5A=[or_xc_5A or_xc_5];
        aor_xcA=[aor_xcA aor_xc];
        
        %%%%Calculate MSE
        abs_or_ckf_1_1(1,t+1,expt)=(xA(1,t+1)-or_xc_1A(1,t+1))^2;
        abs_or_ckf_1_2(1,t+1,expt)=(xA(2,t+1)-or_xc_1A(2,t+1))^2;
        abs_or_ckf_1_3(1,t+1,expt)=(xA(3,t+1)-or_xc_1A(3,t+1))^2;
        abs_or_ckf_1_4(1,t+1,expt)=(xA(4,t+1)-or_xc_1A(4,t+1))^2;
        abs_or_ckf_1_5(1,t+1,expt)=(xA(5,t+1)-or_xc_1A(5,t+1))^2;
        
        abs_or_ckf_2_1(1,t+1,expt)=(xA(1,t+1)-or_xc_2A(1,t+1))^2;
        abs_or_ckf_2_2(1,t+1,expt)=(xA(2,t+1)-or_xc_2A(2,t+1))^2;
        abs_or_ckf_2_3(1,t+1,expt)=(xA(3,t+1)-or_xc_2A(3,t+1))^2;
        abs_or_ckf_2_4(1,t+1,expt)=(xA(4,t+1)-or_xc_2A(4,t+1))^2;
        abs_or_ckf_2_5(1,t+1,expt)=(xA(5,t+1)-or_xc_2A(5,t+1))^2;

        abs_or_ckf_3_1(1,t+1,expt)=(xA(1,t+1)-or_xc_3A(1,t+1))^2;
        abs_or_ckf_3_2(1,t+1,expt)=(xA(2,t+1)-or_xc_3A(2,t+1))^2;
        abs_or_ckf_3_3(1,t+1,expt)=(xA(3,t+1)-or_xc_3A(3,t+1))^2;
        abs_or_ckf_3_4(1,t+1,expt)=(xA(4,t+1)-or_xc_3A(4,t+1))^2;
        abs_or_ckf_3_5(1,t+1,expt)=(xA(5,t+1)-or_xc_3A(5,t+1))^2;
        
        abs_or_ckf_4_1(1,t+1,expt)=(xA(1,t+1)-or_xc_4A(1,t+1))^2;
        abs_or_ckf_4_2(1,t+1,expt)=(xA(2,t+1)-or_xc_4A(2,t+1))^2;
        abs_or_ckf_4_3(1,t+1,expt)=(xA(3,t+1)-or_xc_4A(3,t+1))^2;
        abs_or_ckf_4_4(1,t+1,expt)=(xA(4,t+1)-or_xc_4A(4,t+1))^2;
        abs_or_ckf_4_5(1,t+1,expt)=(xA(5,t+1)-or_xc_4A(5,t+1))^2;
        
        abs_or_ckf_5_1(1,t+1,expt)=(xA(1,t+1)-or_xc_5A(1,t+1))^2;
        abs_or_ckf_5_2(1,t+1,expt)=(xA(2,t+1)-or_xc_5A(2,t+1))^2;
        abs_or_ckf_5_3(1,t+1,expt)=(xA(3,t+1)-or_xc_5A(3,t+1))^2;
        abs_or_ckf_5_4(1,t+1,expt)=(xA(4,t+1)-or_xc_5A(4,t+1))^2;
        abs_or_ckf_5_5(1,t+1,expt)=(xA(5,t+1)-or_xc_5A(5,t+1))^2;

        abs_aor_ckf_1(1,t+1,expt)=(xA(1,t+1)-aor_xcA(1,t+1))^2;
        abs_aor_ckf_2(1,t+1,expt)=(xA(2,t+1)-aor_xcA(2,t+1))^2;
        abs_aor_ckf_3(1,t+1,expt)=(xA(3,t+1)-aor_xcA(3,t+1))^2;
        abs_aor_ckf_4(1,t+1,expt)=(xA(4,t+1)-aor_xcA(4,t+1))^2;
        abs_aor_ckf_5(1,t+1,expt)=(xA(5,t+1)-aor_xcA(5,t+1))^2;

    end

end

%%%%
abs_or_ckf_1_1=abs_or_ckf_1_1+abs_or_ckf_1_3;
abs_or_ckf_1_2=abs_or_ckf_1_2+abs_or_ckf_1_4;
abs_or_ckf_1_3=abs_or_ckf_1_5;

abs_or_ckf_2_1=abs_or_ckf_2_1+abs_or_ckf_2_3;
abs_or_ckf_2_2=abs_or_ckf_2_2+abs_or_ckf_2_4;
abs_or_ckf_2_3=abs_or_ckf_2_5;

abs_or_ckf_3_1=abs_or_ckf_3_1+abs_or_ckf_3_3;
abs_or_ckf_3_2=abs_or_ckf_3_2+abs_or_ckf_3_4;
abs_or_ckf_3_3=abs_or_ckf_3_5;

abs_or_ckf_4_1=abs_or_ckf_4_1+abs_or_ckf_4_3;
abs_or_ckf_4_2=abs_or_ckf_4_2+abs_or_ckf_4_4;
abs_or_ckf_4_3=abs_or_ckf_4_5;

abs_or_ckf_5_1=abs_or_ckf_5_1+abs_or_ckf_5_3;
abs_or_ckf_5_2=abs_or_ckf_5_2+abs_or_ckf_5_4;
abs_or_ckf_5_3=abs_or_ckf_5_5;

abs_aor_ckf_1=abs_aor_ckf_1+abs_aor_ckf_3;
abs_aor_ckf_2=abs_aor_ckf_2+abs_aor_ckf_4;
abs_aor_ckf_3=abs_aor_ckf_5;

%%%%Calculate RMSE
abs_or_ckf_1_1=sqrt(mean(abs_or_ckf_1_1,3));
abs_or_ckf_1_2=sqrt(mean(abs_or_ckf_1_2,3));
abs_or_ckf_1_3=sqrt(mean(abs_or_ckf_1_3,3));

abs_or_ckf_2_1=sqrt(mean(abs_or_ckf_2_1,3));
abs_or_ckf_2_2=sqrt(mean(abs_or_ckf_2_2,3));
abs_or_ckf_2_3=sqrt(mean(abs_or_ckf_2_3,3));

abs_or_ckf_3_1=sqrt(mean(abs_or_ckf_3_1,3));
abs_or_ckf_3_2=sqrt(mean(abs_or_ckf_3_2,3));
abs_or_ckf_3_3=sqrt(mean(abs_or_ckf_3_3,3));

abs_or_ckf_4_1=sqrt(mean(abs_or_ckf_4_1,3));
abs_or_ckf_4_2=sqrt(mean(abs_or_ckf_4_2,3));
abs_or_ckf_4_3=sqrt(mean(abs_or_ckf_4_3,3));

abs_or_ckf_5_1=sqrt(mean(abs_or_ckf_5_1,3));
abs_or_ckf_5_2=sqrt(mean(abs_or_ckf_5_2,3));
abs_or_ckf_5_3=sqrt(mean(abs_or_ckf_5_3,3));

abs_aor_ckf_1=sqrt(mean(abs_aor_ckf_1,3));
abs_aor_ckf_2=sqrt(mean(abs_aor_ckf_2,3));
abs_aor_ckf_3=sqrt(mean(abs_aor_ckf_3,3));

%%%%Plot figure
figure;
j=0:Nk;
plot(j,abs_or_ckf_1_1(1,:),'-k',j,abs_or_ckf_2_1(1,:),'-b',j,abs_or_ckf_3_1(1,:),'-g',j,abs_or_ckf_4_1(1,:),'-c',j,abs_or_ckf_5_1(1,:),'-m',j,abs_aor_ckf_1(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
legend('Existing outlier robust UKF (v=1)','Existing outlier robust UKF (v=2)','Existing outlier robust UKF (v=3)','Existing outlier robust UKF (v=4)','Existing outlier robust UKF (v=5)','The proposed filter'); 

figure;
j=0:Nk;
plot(j,abs_or_ckf_1_2(1,:),'-k',j,abs_or_ckf_2_2(1,:),'-b',j,abs_or_ckf_3_2(1,:),'-g',j,abs_or_ckf_4_2(1,:),'-c',j,abs_or_ckf_5_2(1,:),'-m',j,abs_aor_ckf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('Existing outlier robust UKF (v=1)','Existing outlier robust UKF (v=2)','Existing outlier robust UKF (v=3)','Existing outlier robust UKF (v=4)','Existing outlier robust UKF (v=5)','The proposed filter'); 

figure;
j=0:Nk;
plot(j,abs_or_ckf_1_3(1,:)./raddeg,'-k',j,abs_or_ckf_2_3(1,:)./raddeg,'-b',j,abs_or_ckf_3_3(1,:)./raddeg,'-g',j,abs_or_ckf_4_3(1,:)./raddeg,'-c',j,abs_or_ckf_5_3(1,:)./raddeg,'-m',j,abs_aor_ckf_3(1,:)./raddeg,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{ome} (Deg/s)');
legend('Existing outlier robust UKF (v=1)','Existing outlier robust UKF (v=2)','Existing outlier robust UKF (v=3)','Existing outlier robust UKF (v=4)','Existing outlier robust UKF (v=5)','The proposed filter'); 
