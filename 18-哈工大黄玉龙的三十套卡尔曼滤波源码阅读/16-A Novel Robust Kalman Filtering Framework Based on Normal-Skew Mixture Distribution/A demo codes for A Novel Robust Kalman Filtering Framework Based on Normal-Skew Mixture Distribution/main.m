%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab demo code for the paper
% Bai M, Huang Y, Chen B, et al. A Novel Robust Kalman Filtering Framework Based on Normal-Skew Mixture Distribution. 
% IEEE Transactions on Systems, Man, and Cybernetics: Systems, 2021.,"  
% URL: https://ieeexplore.ieee.org/document/9739998

% If you use our code in your publication, please cite the above paper.
% Demo code written by Mingming Bai.
% Email: mingming.bai@hrbeu.edu.cn
% Looking forward to your feedback.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%
profile on; 
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%Model parameters
nxp=100;
nx=4;
nz=2;
T=1;        
q=0.1;
r=10;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=400;

%%%%Parameter selections
U1=100;      %%%%%%Parameter of state outlier
U2=100;      %%%%%%Parameter of measurement outlier
N=20;        %%%%%%Maximum number of iterations

for expt = 1:nxp
    
    fprintf('MC number = %d\n',expt); 

    %%%%Initial values    
    x=[0;0;10;10];                          
    P=diag([100 100 100 100]);    
    Skk=utchol(P);                        

    %%%%KF (Kalman filter)   KFTNCM
    xf=x+Skk*randn(nx,1);                   
    Pf=P;

    %%%%KF (Kalman filter)   Optimal Kalman filter
    xtf=xf;                 
    Ptf=P;

    %%%%RSTKF
    xaprwviv=xf;
    Paprwviv=Pf;
    tao_p=5;
    tao_r=5;
    a=5;
    b=1;
    c=5;
    d=1;
   v_stgt_1=5;
    v_stgt_2=5; 
    w_sl_ghsst=1.0;
    v_sl_ghsst=5;
    beta1_bar=[0;0;0;0];          
    sigama1=0;               
    tao1=5;
    tao2=5;

    %%%%The proposed filter 
    xapiv_vg_ghvg=xf;
    Papiv_vg_ghvg=Pf;
    xapiv_st_ghvg=xf;
    Papiv_st_ghvg=Pf;
    v_slgt_1=5; 
    v_slgt_2=5;
    v_vggt_1=0.5;
    v_vggt_2=5;
    e=0.85; %%sensitive 
    g=0.85; %%sensitive 
    f0=5;
    u0=5;
    beta2_bar=1.5*ones(nz,1);
    sigama2=1;

    %%%%Save data
    xA=x;
    xfA=xf;
    xtfA=xtf;
    xapiv_st_ghvg_A=xapiv_st_ghvg;
    xapiv_vg_ghvg_A=xapiv_vg_ghvg;

    for t=1:ts

        %%%%%%%Simulate non-stationary noise
        %%%%%%%Gaussian noise
        if t<=100
            p1=1;
            p2=1;
        end
        %%%%%%%Slightly heavy-tailed noise
        if (t>100)&&(t<=200)
            p1=0.90;
            p2=0.90; 
        end
        %%%%%%%Moderately heavy-tailed noise
        if (t>200)&&(t<=300)
            p1=0.80;
            p2=0.80;
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
            wk=utchol(Q)*randn(nx,1);
        else
            Q=U1*Q0;
            wk=utchol(Q)*randn(nx,1);
        end

        R=R0;

        if test2<=p2 %%
            vk=utchol(R0)*randn(nz,1); 
        else  
            vk=skew_noise(diag([5.0;5.0]),R0,5);
        end

        %%%%True noise covariance matrices
        D_Q=p1*Q0+(1-p1)*U1*Q0;
        D_R=R0;  

        %%%%Simulate true state and measurement
        x=F*x+wk;
        z=H*x+vk;

        %%%%Filtering
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);

        [xapiv_st_ghvg,Papiv_st_ghvg]=apivbkf_st_ghvg(xapiv_st_ghvg,Papiv_st_ghvg,F,H,z,Q0,R0,...
                                        f0,u0,e,g,v_stgt_1,v_stgt_2,beta2_bar,sigama2,N);

        [xapiv_vg_ghvg,Papiv_vg_ghvg]=apivbkf_vg_ghvg(xapiv_vg_ghvg,Papiv_vg_ghvg,F,H,z,Q0,R0,...
                                        f0,u0,e,g,v_vggt_1,v_vggt_2,beta2_bar,sigama2,N);

        %%%%Save data
        xA=[xA x];
        xfA=[xfA xf];
        xapiv_st_ghvg_A=[xapiv_st_ghvg_A xapiv_st_ghvg];
        xapiv_vg_ghvg_A=[xapiv_vg_ghvg_A xapiv_vg_ghvg];

        %%%%MSE calculation
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;

        mse_apivbkf_stvg_1(1,t,expt)=(xA(1,t+1)-xapiv_st_ghvg_A(1,t+1))^2+(xA(2,t+1)-xapiv_st_ghvg_A(2,t+1))^2;
        mse_apivbkf_stvg_2(1,t,expt)=(xA(3,t+1)-xapiv_st_ghvg_A(3,t+1))^2+(xA(4,t+1)-xapiv_st_ghvg_A(4,t+1))^2;

        mse_apivbkf_vgvg_1(1,t,expt)=(xA(1,t+1)-xapiv_vg_ghvg_A(1,t+1))^2+(xA(2,t+1)-xapiv_vg_ghvg_A(2,t+1))^2;
        mse_apivbkf_vgvg_2(1,t,expt)=(xA(3,t+1)-xapiv_vg_ghvg_A(3,t+1))^2+(xA(4,t+1)-xapiv_vg_ghvg_A(4,t+1))^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE calculation
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_apivbkf_stvg_1=sqrt(mean(mse_apivbkf_stvg_1,3));
rmse_apivbkf_stvg_2=sqrt(mean(mse_apivbkf_stvg_2,3));

rmse_apivbkf_vgvg_1=sqrt(mean(mse_apivbkf_vgvg_1,3));
rmse_apivbkf_vgvg_2=sqrt(mean(mse_apivbkf_vgvg_2,3));

%%%%%%%RMSE smooth
srmse_kf_1=smooth(rmse_kf_1,10);
srmse_kf_2=smooth(rmse_kf_2,10);
      
srmse_apivbkf_stvg_1=smooth(rmse_apivbkf_stvg_1,10);
srmse_apivbkf_stvg_2=smooth(rmse_apivbkf_stvg_2,10);
             
srmse_apivbkf_vgvg_1=smooth(rmse_apivbkf_vgvg_1,10);
srmse_apivbkf_vgvg_2=smooth(rmse_apivbkf_vgvg_2,10);
    
%%%%%%%Smoothing RMSE
%%%%%%%Smoothing RMSE
figure;
j = 1:ts;
plot(j,srmse_kf_1,'-ok','MarkerFaceColor','k')
hold on;
plot(j,srmse_apivbkf_stvg_1,'-sg','MarkerFaceColor','g')
hold on;
plot(j,srmse_apivbkf_vgvg_1,'-sb','MarkerFaceColor','b')
xlabel('Number of iteration');
ylabel('ARMSE_{pos} (m)');
legend('KFNCM','The proposed RKF5','The proposed RKF6'); 
axis tight;

figure;
j = 1:ts;
plot(j,srmse_kf_2,'-ok','MarkerFaceColor','k')
hold on;
plot(j,srmse_apivbkf_stvg_2,'-sg','MarkerFaceColor','g')
hold on;
plot(j,srmse_apivbkf_vgvg_2,'-sb','MarkerFaceColor','b')
xlabel('Number of iteration');
ylabel('ARMSE_{vel} (m/s)');
legend('KFNCM','The proposed RKF5','The proposed RKF6'); 
axis tight;


