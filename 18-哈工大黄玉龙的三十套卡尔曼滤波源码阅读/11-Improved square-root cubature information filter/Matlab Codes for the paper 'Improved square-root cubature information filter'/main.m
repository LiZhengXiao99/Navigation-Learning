%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%%%%Model parameters
raddeg=pi/180;
nx=5;
nz=2;
nxp=200; 
ts=50;                      %%%%%%Simulation step
ns=10;                      %%%%%%The nonumber of sensor

%%%%%%%%%%%%%%%%%%%%%%
%%%%%%Important parameters
Omega_0=-3*raddeg;          %%%%%%Initial turn rate
T=3;                        %%%%%%Sampling interval
%%%%%%%%%%%%%%%%%%%%%%

%%%%%%Noise parameters
q1=0.1;
q2=1.75e-4;
Cr=10;
Co=sqrt(10)*1e-3;
M=[T^3/3 T^2/2;T^2/2 T];

%%%%%%Noise covariance matrices
Q=[q1*M zeros(2,2) zeros(2,1);zeros(2,2) q1*M zeros(2,1);zeros(1,2) zeros(1,2) q2*T];
R=diag([Cr^2 Co^2]);
SQ=utchol(Q);
SR=diag([Cr Co]);

for expt=1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%Initialization
    x=[1000;300;1000;0;Omega_0];
    P=diag([100 10 100 10 1e-4]);
    Skk=utchol(P);
    
    %%%%%%Generate the localizations of sensor randomly
    Xs=repmat(-4000,1,ns)+8000*rand(1,ns);
    Ys=repmat(-4000,1,ns)+8000*rand(1,ns);
    z=zeros(nz,ns);

    %%%%Existing CIF
    xc=x+Skk*randn(nx,1);
    Pc=P;
    
    %%%%Existing SCIF
    xsc=xc;
    Psc=Skk;
    
    %%%%The proposed SCIF
    New_xsc=xc;
    New_Psc=Skk;

    %%%%Save data
    xA=x;
    xscA=xsc;
    New_xscA=New_xsc;
    
    for t=1:ts
        
        %%%%%%Simulate true state and measurement
        x=ProssEq(x,T)+SQ*randn(nx,1);
            
        z=MstEq(x,Xs,Ys)+SR*randn(nz,ns);   %%%%nz*ns

        %%%%%%Information filtering
        [xsc,Psc]=scif(xsc,Psc,z,SQ,SR,Xs,Ys,T);

        [New_xsc,New_Psc] = New_scif(New_xsc,New_Psc,z,SQ,SR,Xs,Ys,T);
        
        %%%%Save data
        xA=[xA x];
        xscA=[xscA xsc];
        New_xscA=[New_xscA New_xsc];

        %%%%Calculate MSEs
        mse_scif_1(1,t,expt)=(xA(1,t)-xscA(1,t))^2+(xA(3,t)-xscA(3,t))^2;
        mse_scif_2(1,t,expt)=(xA(2,t)-xscA(2,t))^2+(xA(4,t)-xscA(4,t))^2;
        mse_scif_3(1,t,expt)=(xA(5,t)-xscA(5,t))^2;
 
        mse_New_scif_1(1,t,expt)=(xA(1,t)-New_xscA(1,t))^2+(xA(3,t)-New_xscA(3,t))^2;
        mse_New_scif_2(1,t,expt)=(xA(2,t)-New_xscA(2,t))^2+(xA(4,t)-New_xscA(4,t))^2;
        mse_New_scif_3(1,t,expt)=(xA(5,t)-New_xscA(5,t))^2;

    end

end

%%%%%%%Calculate RMSEs
rmse_scif_1=sqrt(mean(mse_scif_1,3));
rmse_scif_2=sqrt(mean(mse_scif_2,3));
rmse_scif_3=sqrt(mean(mse_scif_3,3));
 
rmse_New_scif_1=sqrt(mean(mse_New_scif_1,3));
rmse_New_scif_2=sqrt(mean(mse_New_scif_2,3));
rmse_New_scif_3=sqrt(mean(mse_New_scif_3,3));
 
%%%%%%%
figure;
j = 1:ts;
plot(j*T,rmse_scif_1(1,:),'--k',j*T,rmse_New_scif_1(1,:),'-k','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE_{pos}(m)');
legend('The existing SCIF','The proposed SCIF');
 
figure;
j = 1:ts;
plot(j*T,rmse_scif_2(1,:),'--k',j*T,rmse_New_scif_2(1,:),'-k','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE_{vel}(m/s)');
legend('The existing SCIF','The proposed SCIF');
 
figure;
j = 1:ts;
plot(j*T,rmse_scif_3(1,:)./raddeg,'--k',j*T,rmse_New_scif_3(1,:)./raddeg,'-k','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE_{ome}(Deg/s)');
legend('The existing SCIF','The proposed SCIF');

