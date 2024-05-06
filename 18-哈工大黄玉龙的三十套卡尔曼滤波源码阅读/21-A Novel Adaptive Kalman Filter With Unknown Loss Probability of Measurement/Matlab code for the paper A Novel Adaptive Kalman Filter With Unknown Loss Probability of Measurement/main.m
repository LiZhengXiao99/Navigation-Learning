%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%Model parameters
raddeg=pi/180;
nx=2;
nz=1;
nxp=1000;               %%%%The number of Monte Carlo simulation 
ts=100;                    %%%%Simulation time
T=0.01;                    %%%%Sampling time
Nk=ts/T;
T1=1;        
q=1;
r=150;
Q=T1*eye(2)*q;
SQ=utchol(Q);
R=r*eye(1);
SR=utchol(R); 
F=[0.6 0.4;0.1 0.9];
H=[1 -2];

%%%%Parameter selections  
Nm=50;
rou=1-exp(-5);

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%Initial values
    x=[10;10];                          
    P=diag([100 100]);
    Skk=utchol(P);
    
    x0=x+Skk*randn(nx,1);
    P0=P;
    
    %%%%IKF (Kalman filter with known sensor measurement loss)
    xi=x0;
    Pi=P0;
    
    %%%%The proposed filter (Adaptive Kalman filter with unknown probability of measurement loss) 
    xakf=x0;
    Pakf=P0;
    alfa=5;  
    beta=5;
    
    %%%%Save data
    xA=x;
    xiA=xi;
    xakfA=xakf;
    ppA=0.5;
    p0A=[];
    rk0A=[];
    yA=[];

    for t=1:Nk
        
        %%%%%%%Simulate the probability of measurement loss
        if t<Nk/3
            p0=0.1;
        elseif t>2*Nk/3
            p0=0.1;
        else
            p0=0.3;
        end
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        alfa_1=Bernoulli(p0);
        rk0=(1-alfa_1);
        y=rk0*H*x+SR*randn(nz,1);
        
        %%%%Save data
        p0A=[p0A p0];
        rk0A=[rk0A rk0];
        xA=[xA x];
        yA=[yA y];
        p_0(t)=p0A(t);
        
    end
    
    for t=1:Nk
        
        rk0=rk0A(t);
        y=yA(t);
        
        %%%%Filtering
        [xi,Pi] = ikf(xi,Pi,F,H,y,Q,R,rk0);

        [xakf,Pakf,alfa,beta] = akf_upml(xakf,Pakf,F,H,y,Q,R,alfa,beta,rou,Nm);
           
        %%%%Save state estimate
        xA=[xA x];  
        xiA=[xiA xi];                      %%%%IKF 
        xakfA=[xakfA xakf];          %%%%The proposed filter
        
        %%%%
        pp=alfa/(alfa+beta);         %%%%Calculate the estimate of the loss probability
        ppA=[ppA pp];                 
        
        %%%%Calculate MSE        
        abs_ikf_1(1,t,expt)=(xA(1,t+1)-xiA(1,t+1))^2;
        abs_ikf_2(1,t,expt)=(xA(2,t+1)-xiA(2,t+1))^2;
        
        abs_akf_1(1,t,expt)=(xA(1,t+1)-xakfA(1,t+1))^2;
        abs_akf_2(1,t,expt)=(xA(2,t+1)-xakfA(2,t+1))^2;

        %%%%
        p_vb(1,t,expt)=ppA(t);

    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%MSE calculation
mse_ikf=abs_ikf_1+abs_ikf_2;
mse_akf=abs_akf_1+abs_akf_2;
 
%%%%RMSE calculation
rmse_ikf=sqrt(mean(mse_ikf,3));
rmse_akf=sqrt(mean(mse_akf,3));

%%%%The estimate of the loss probability calculation
pp_vb=mean(p_vb,3);

%%%%RMSEs of the IKF and the proposed filter
figure;
j = 1:Nk;
plot(j*T,rmse_ikf(1,:),'-c',j*T,rmse_akf(1,:),'-r','linewidth',2.5);
xlabel('time(s)');
ylabel('RMSE_{pos}(m)');
legend('IKF','The proposed filter');

%%%%The true and estimated loss probability
figure;
j = 1:Nk;
plot(j*T,p_0,'-c',j*T,pp_vb,'-r','linewidth',2.5);
xlabel('time(s)');
ylabel('The loss probablilities');
legend('The true loss probability','The estimate of the loss probability p_{ml}');
