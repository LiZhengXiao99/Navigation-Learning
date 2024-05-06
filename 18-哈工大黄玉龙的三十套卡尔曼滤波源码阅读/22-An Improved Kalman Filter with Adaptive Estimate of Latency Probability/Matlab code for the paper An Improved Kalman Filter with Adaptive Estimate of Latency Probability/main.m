profile on
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%%%%%Model parameters
nx=4;
nz=2;

nxp=250;%%%%%%%%Monte Carlo simulation
ts=10000;%%%%%%%%Timed
T=1;
Nk=ts/T;

q=1;
r=100;
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);

F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
N=10;   
rou=0.99;

for expt = 1:nxp
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%Initial values
    x=[0;0;0;0];                          
    P=diag([10000 10000 100 100]);
    Skk=utchol(P);
    z=[];
   
    %%%%%%%%KF(Kalman Filter)
    xc=x+Skk*randn(nx,1);
    Pc=P;
    
    %%%%%%%%DKF(Delayed Kalman Filter)
    dxc=xc;
    dPc=Pc;
    xa=[dxc;zeros(nz,1)];
    Pa=[dPc zeros(nx,nz);zeros(nz,nx) R0];

    %%%%%%%%The proposed filter DVBKF(the Improved Kalman Filter with Adaptive Estimate of Latency probability)
    dvbxc=xc;
    dvbpc=Pc;
    alfa0=10;
    beta0=10;
    
    %%%%%%%%Save data
    zA=z;
    xA=x;
    xcA=xc;
    dxcA=dxc;
    dvbxcA=dvbxc;
    
    alfaA=alfa0;
    betaA=beta0;
    
    ppA=0.5;
    p1A=[];

    for t=1:Nk
        %%%%%%%%Simulate the probability of measurement delayed
        if t<Nk/4
            
            p0=0.2;
            
        elseif t>3*Nk/4
            
            p0=0.3;
        else
            
            p0=0.5;
        end
        
        Q=Q0;
        R=R0;

        SQ=utchol(Q);    
        SR=utchol(R);  
       
        %%%%%%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);
        zA=[zA z];

        if t<=1
            p1=0;
            y=zA(:,t);
        end
        
        if t>=2
            p1=p0;
           
            alfa_1=Bernoulli(p1);
            
            rk0=alfa_1;
            rk1=(1-alfa_1);
            
            y=rk0*zA(:,t)+rk1*zA(:,t-1);
        end
        
        p_one=0.5;%%%%%The nominal latency probability of the delayed KF
        
        %%%%%%%%Filtering
        [xc,Pc] = kf(xc,Pc,F,H,y,Q,R);
        [xa,Pa] = dkf(xa,Pa,F,H,y,Q,R,p_one);
        [dvbxc,dvbpc,alfa0,beta0] = dvbkf(dvbxc,dvbpc,F,H,y,Q,R,alfa0,beta0,rou,N);
       
        %%%%%%%%Save data
        xA=[xA x];
        xcA=[xcA xc];
        dxc=xa(1:nx,:);
        dxcA=[dxcA dxc];
        dvbxcA=[dvbxcA dvbxc];
     
        alfaA=[alfaA alfa0];
        betaA=[betaA beta0];
        
        pp=alfa0/(alfa0+beta0);
        ppA=[ppA pp];
        p_vb(1,t,expt)=ppA(t);
        p1A=[p1A p1];
        p_1(t)=p1A(t);

        abs_kf_1(1,t,expt)=(xA(1,t)-xcA(1,t))^2;
        abs_kf_2(1,t,expt)=(xA(2,t)-xcA(2,t))^2;
        abs_kf_3(1,t,expt)=(xA(3,t)-xcA(3,t))^2;
        abs_kf_4(1,t,expt)=(xA(4,t)-xcA(4,t))^2;
        
        abs_dkf_1(1,t,expt)=(xA(1,t)-dxcA(1,t))^2;
        abs_dkf_2(1,t,expt)=(xA(2,t)-dxcA(2,t))^2;
        abs_dkf_3(1,t,expt)=(xA(3,t)-dxcA(3,t))^2;
        abs_dkf_4(1,t,expt)=(xA(4,t)-dxcA(4,t))^2;

        abs_dvbkf_1(1,t,expt)=(xA(1,t)-dvbxcA(1,t))^2;
        abs_dvbkf_2(1,t,expt)=(xA(2,t)-dvbxcA(2,t))^2;
        abs_dvbkf_3(1,t,expt)=(xA(3,t)-dvbxcA(3,t))^2;
        abs_dvbkf_4(1,t,expt)=(xA(4,t)-dvbxcA(4,t))^2;
     end
end

%%%%%%%%The estimate of the latency probability calculation
pp_vb=mean(p_vb,3);

%%%%%%%%MSE calculation
abs_kf_1=abs_kf_1+abs_kf_2;
abs_kf_2=abs_kf_3+abs_kf_4;

abs_dkf_1=abs_dkf_1+abs_dkf_2;
abs_dkf_2=abs_dkf_3+abs_dkf_4;

abs_dvbkf_1=abs_dvbkf_1+abs_dvbkf_2;
abs_dvbkf_2=abs_dvbkf_3+abs_dvbkf_4;

%%%%%%%%MSE calculation
rmse_kf_1=sqrt(mean(abs_kf_1,3));
rmse_kf_2=sqrt(mean(abs_kf_2,3));

rmse_dkf_1=sqrt(mean(abs_dkf_1,3));
rmse_dkf_2=sqrt(mean(abs_dkf_2,3));

rmse_dvbkf_1=sqrt(mean(abs_dvbkf_1,3));
rmse_dvbkf_2=sqrt(mean(abs_dvbkf_2,3));

%%%%%%%%RMSE calculation
armse_kf_1=sqrt(mean(mean(abs_kf_1,3),2));
armse_kf_2=sqrt(mean(mean(abs_kf_2,3),2));


armse_dkf_1=sqrt(mean(mean(abs_dkf_1,3),2));
armse_dkf_2=sqrt(mean(mean(abs_dkf_2,3),2));

armse_dvbkf_1=sqrt(mean(mean(abs_dvbkf_1,3),2));
armse_dvbkf_2=sqrt(mean(mean(abs_dvbkf_2,3),2));

%%%%%%%%RMSE with different filters
figure;
j = 1:Nk;
plot(j*T,p_1,'-r',j*T,pp_vb,'-g','linewidth',2.5);
xlabel('time(s)');
ylabel('Delay Probability');
legend('True','The proposed');

%%%%%%%%The true and estimated latency probability
figure
j = 1:Nk;
subplot(2,1,1);
plot(j,rmse_kf_1(1,:),'-b',j,rmse_dkf_1(1,:),'-g',j,rmse_dvbkf_1(1,:),'-r');
xlabel('time(s)');
ylabel('RMSEs of Position(m)');
subplot(2,1,2);
plot(j,rmse_kf_2(1,:),'-b',j,rmse_dkf_2(1,:),'-g',j,rmse_dvbkf_2(1,:),'-r');
xlabel('time(s)');
ylabel('RMSEs of Velocity(m/s)');
legend('KF','GA','The proposed');

profile viewer
