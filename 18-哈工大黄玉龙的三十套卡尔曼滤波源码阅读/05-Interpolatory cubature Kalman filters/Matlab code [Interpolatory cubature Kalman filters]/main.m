%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));    

%%%%%
nxp=50;
T=2000;
n=3;
Q=0.1*eye(n);
SQ=utchol(Q);
R =1;
SR=sqrt(R);

%%%%Filter parameters
%%%%First set of parameters
lamda_10=sqrt(5-10^(1/2));
lamda_20=sqrt(5+10^(1/2));

%%%%Second set of parameters
lamda_11=sqrt(5+10^(1/2));
lamda_21=sqrt(5-10^(1/2));

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%
    x=[-0.7;1;1];
    P=eye(n);
    Skk=utchol(P);
    
    %%%%CKF
    xc=x+Skk*randn(n,1); 
    Pc=P;
   
    %%%%High-degree ICKF
    high_i_xc0=xc;
    high_i_Pc0=P;

    high_i_xc1=xc;
    high_i_Pc1=P;

    %%%%Save data
    xA=x;
    xcA=xc;
    high_i_xcA0=high_i_xc0;
    high_i_xcA1=high_i_xc1;

    for t=1:T
        
        x=ProssEq(x)+SQ*randn(n,1);
        z=MstEq(x)+SR*randn;
        
        %%%%Filtering
        [xc,Pc]=ckf(xc,Pc,z,Q,R);
        
        [high_i_xc0,high_i_Pc0] = high_ickf(high_i_xc0,high_i_Pc0,z,Q,R,lamda_10,lamda_20);
        
        [high_i_xc1,high_i_Pc1] = high_ickf(high_i_xc1,high_i_Pc1,z,Q,R,lamda_11,lamda_21);
        
        %%%%Save data
        xA=[xA x];
        xcA=[xcA xc];
        high_i_xcA0=[high_i_xcA0 high_i_xc0];
        high_i_xcA1=[high_i_xcA1 high_i_xc1];
        
        %%%%Calculate MSE
        abs_ckf_1(1,t,expt)=(xA(1,t)-xcA(1,t))^2;
        abs_ckf_2(1,t,expt)=(xA(2,t)-xcA(2,t))^2;
        abs_ckf_3(1,t,expt)=(xA(3,t)-xcA(3,t))^2;

        abs_high_i_ckf0_1(1,t,expt)=(xA(1,t)-high_i_xcA0(1,t))^2;
        abs_high_i_ckf0_2(1,t,expt)=(xA(2,t)-high_i_xcA0(2,t))^2;
        abs_high_i_ckf0_3(1,t,expt)=(xA(3,t)-high_i_xcA0(3,t))^2;
        
        abs_high_i_ckf1_1(1,t,expt)=(xA(1,t)-high_i_xcA1(1,t))^2;
        abs_high_i_ckf1_2(1,t,expt)=(xA(2,t)-high_i_xcA1(2,t))^2;
        abs_high_i_ckf1_3(1,t,expt)=(xA(3,t)-high_i_xcA1(3,t))^2;

    end
    
end

%%%%Calculate RMSE
abs_ckf_1=sqrt(mean(abs_ckf_1,2));
abs_ckf_2=sqrt(mean(abs_ckf_2,2));
abs_ckf_3=sqrt(mean(abs_ckf_3,2));

abs_high_i_ckf0_1=sqrt(mean(abs_high_i_ckf0_1,2));
abs_high_i_ckf0_2=sqrt(mean(abs_high_i_ckf0_2,2));
abs_high_i_ckf0_3=sqrt(mean(abs_high_i_ckf0_3,2));
       
abs_high_i_ckf1_1=sqrt(mean(abs_high_i_ckf1_1,2));
abs_high_i_ckf1_2=sqrt(mean(abs_high_i_ckf1_2,2));
abs_high_i_ckf1_3=sqrt(mean(abs_high_i_ckf1_3,2));


%%%%RMSE curves
figure;
j = 1:nxp;
plot(j,abs_ckf_1(1,:),'-pk',j,abs_high_i_ckf0_1(1,:),'-vr',j,abs_high_i_ckf1_1(1,:),'-or');
legend('3rd-CKF','The proposed 5th-ICKF (\lambda_1=1.356,\lambda_2=2.857)','The proposed 5th-ICKF (\lambda_1=2.857,\lambda_2=1.356)');
xlabel('Runs');
ylabel('RMSE of x_!');
hold on;

figure;
j = 1:nxp;
plot(j,abs_ckf_2(1,:),'-pk',j,abs_high_i_ckf0_2(1,:),'-vr',j,abs_high_i_ckf1_2(1,:),'-or');
legend('3rd-CKF','The proposed 5th-ICKF (\lambda_1=1.356,\lambda_2=2.857)','The proposed 5th-ICKF (\lambda_1=2.857,\lambda_2=1.356)');
xlabel('Runs');
ylabel('RMSE of x_2');
hold on;

figure;
j = 1:nxp;
plot(j,abs_ckf_3(1,:),'-pk',j,abs_high_i_ckf0_3(1,:),'-vr',j,abs_high_i_ckf1_3(1,:),'-or');
legend('3rd-CKF','The proposed 5th-ICKF (\lambda_1=1.356,\lambda_2=2.857)','The proposed 5th-ICKF (\lambda_1=2.857,\lambda_2=1.356)');
xlabel('Runs');
ylabel('RMSE of x_3');
hold on;


