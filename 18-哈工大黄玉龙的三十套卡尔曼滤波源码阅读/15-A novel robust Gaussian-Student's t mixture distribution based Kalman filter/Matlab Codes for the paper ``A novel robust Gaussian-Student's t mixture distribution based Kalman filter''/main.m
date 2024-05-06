%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%Model parameters
nxp=1000;
nx=1;
nz=1;
F=0.5;
H=1;
Q0=100;
R0=100;
ts=400;

%%%%Parameter selections
U1=1000;     %%%%%%Parameter of state outlier
U2=100;      %%%%%%Parameter of measurement outlier
N=50;        %%%%%%Maximum number of iterations
v=5;         %%%%%%Dof parameter  

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%Initial values
    x=100;                          
    P=1000;        
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

    %%%%The proposed filter (IEEE TSP)
    xapiv=xf;
    Papiv=Pf;
    e=0.85;
    f=0.15;
    g=0.85;
    h=0.15;
    
    %%%%Save data
    xA=x;
    xfA=xf;
    xtfA=xtf;
    xaprwvivA=xaprwviv;
    xapivA=xapiv;

    for t=1:ts
        
        %%%%%%%Simulate non-stationary noise
        %%%%%%%Gaussian noise
        if t<=100
            p1=1;
            p2=1;
        end
        %%%%%%%Slightly heavy-tailed noise
        if (t>100)&&(t<=200)
            p1=0.99;
            p2=0.99;
        end
        %%%%%%%Moderately heavy-tailed noise
        if (t>200)&&(t<=300)
            p1=0.95;
            p2=0.95;
        end
        %%%%%%%Gaussian noise
        if t>300
            p1=1;
            p2=1;
        end
        
        %%%%True noise covariance matrices
        D_Q=p1*Q0+(1-p1)*U1*Q0;
        D_R=p2*R0+(1-p2)*U2*R0;

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
        
        %%%%Calculate square-root matrix
        SQ=utchol(Q);    
        SR=utchol(R);    
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);

        %%%%Filtering
        [xf,Pf]=kf(xf,Pf,F,H,z,D_Q,D_R);
        
        [xtf,Ptf]=kf(xtf,Ptf,F,H,z,Q,R);

        [xaprwviv,Paprwviv]=aprwvivbkf(xaprwviv,Paprwviv,F,H,z,Q0,R0,tao_p,tao_r,a,b,c,d,N);

        [xapiv,Papiv]=apivbkf(xapiv,Papiv,F,H,z,Q0,R0,N,e,f,g,h,v,v);

        %%%%Save data
        xA=[xA x];
        xfA=[xfA xf];
        xtfA=[xtfA xtf];
        xaprwvivA=[xaprwvivA xaprwviv];
        xapivA=[xapivA xapiv];
        
        %%%%MSE calculation
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2;
        mse_tkf_1(1,t,expt)=(xA(1,t+1)-xtfA(1,t+1))^2;
        mse_aprwvivbkf_1(1,t,expt)=(xA(1,t+1)-xaprwvivA(1,t+1))^2;
        mse_apivbkf_1(1,t,expt)=(xA(1,t+1)-xapivA(1,t+1))^2;
 
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE calculation
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_tkf_1=sqrt(mean(mse_tkf_1,3));
rmse_aprwvivbkf_1=sqrt(mean(mse_aprwvivbkf_1,3));
rmse_apivbkf_1=sqrt(mean(mse_apivbkf_1,3));

%%%%%%%%%ARMSE calculation (First stage)
armse_kf_11=mean(rmse_kf_1(1:100))
armse_tkf_11=mean(rmse_tkf_1(1:100))
armse_aprwvivbkf_11=mean(rmse_aprwvivbkf_1(1:100))
armse_apivbkf_11=mean(rmse_apivbkf_1(1:100))

%%%%%%%%%ARMSE calculation (Second stage)
armse_kf_12=mean(rmse_kf_1(101:200))
armse_tkf_12=mean(rmse_tkf_1(101:200))
armse_aprwvivbkf_12=mean(rmse_aprwvivbkf_1(101:200))
armse_apivbkf_12=mean(rmse_apivbkf_1(101:200))

%%%%%%%%%ARMSE calculation (Third stage)
armse_kf_13=mean(rmse_kf_1(201:300))
armse_tkf_13=mean(rmse_tkf_1(201:300))
armse_aprwvivbkf_13=mean(rmse_aprwvivbkf_1(201:300))
armse_apivbkf_13=mean(rmse_apivbkf_1(201:300))

%%%%%%%%%ARMSE calculation (Fourth stage)
armse_kf_14=mean(rmse_kf_1(301:400))
armse_tkf_14=mean(rmse_tkf_1(301:400))
armse_aprwvivbkf_14=mean(rmse_aprwvivbkf_1(301:400))
armse_apivbkf_14=mean(rmse_apivbkf_1(301:400))

%%%%%%%RMSE smooth
srmse_kf_1=smooth(rmse_kf_1,10);
srmse_tkf_1=smooth(rmse_tkf_1,10);
srmse_aprwvivbkf_1=smooth(rmse_aprwvivbkf_1,10);
srmse_apivbkf_1=smooth(rmse_apivbkf_1,10);

%%%%%%%Smoothing RMSE
figure;
j=1:ts;
plot(j,srmse_kf_1,'-k',j,srmse_aprwvivbkf_1,'-b',j,srmse_apivbkf_1,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_x');
legend('KFTNCM','RSTKF','The proposed filter');    
