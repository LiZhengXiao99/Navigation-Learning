%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));             
format long;

%%%%%Model parameters
nxp=100;
nx=4;
nz=2;
T=1;       
q=1;
r=100;                                                   
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q1=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R1=r*[1 0.5;0.5 1];
ts=1000;

%%%%%Selections of filtering parameters
N=50;                 
tao_P=5;  
tao_R=5; 
rou=1-exp(-4);
beta=10;
Q_s=10;
alfa=5:5:100;

for expt=1:nxp
    
    fprintf('MC Run in Process=%d\n',expt); 
    
    %%%%%Set the system initial value%%%%%%%%%%%
    x=[100;100;10;10];                     %%%True initial state value 
    P=diag([100 100 100 100]);             %%%Initial estimation error covariance matrix 
    Skk=utchol(P);                         %%%Square-root of initial estimation error covariance matrix
    
    %%%%Nominal measurement noise covariance matrix (R)
    R0=beta*eye(nz);
    
    %%%%Initial state estimate of standard KF    (Kalman filter)
    xf=x+Skk*randn(nx,1);               
    Pf=P;
    
    %%%%Initial state estimate of Kalman filter with true noise covariance matrices    (Optimal Kalman filter)
    xtf=xf;
    Ptf=Pf;

    %%%%Initial state estimate of adaptive methods
    xiv=xf;
    Piv=Pf;

    %%%%
    xA=[];
    ZA=[];
    
    for t=1:ts
    
        %%%%True noise covariance matrices
        Q=Q_s*(6.5+0.5*cos(0.5*2*pi*t/ts))*Q1;
        R=(0.1+0.05*cos(0.5*2*pi*t/ts))*R1;
        
        %%%%Square-root of noise covariance matrices
        SQ=utchol(Q);  
        SR=utchol(R);    
        
        %%%%Simulate true state and measurement
        x=F*x+SQ*randn(nx,1);
        z=H*x+SR*randn(nz,1);
        
        %%%Save data
        xA=[xA x];
        ZA=[ZA z];
        
        %%%%Run optimal Kalman filter
        [xtf,Ptf,Pptf]=kf(xtf,Ptf,F,H,z,Q,R);

        %%%%MSE calculation
        mse_ktf_1(1,t,expt)=(x(1)-xtf(1))^2+(x(2)-xtf(2))^2;
        mse_ktf_2(1,t,expt)=(x(3)-xtf(3))^2+(x(4)-xtf(4))^2;

        %%%%NEES calculation
        nees_ktf(1,t,expt)=(x-xtf)'*inv(Ptf)*(x-xtf);

    end
    
    for i=1:length(alfa)
        
        %%%%Nominal state noise covariance matrix (Q)
        Q0=alfa(i)*eye(nx);
        
        %%%%Initial state estimate of VBAKF-PR
        xapriv=xiv;
        Papriv=Piv;
        uapriv=tao_R;
        Uapriv=tao_R*R0;
        
        %%%%Initial state estimate of proposed SWVAKF
        new_xapriv=xiv;
        new_Papriv=Piv;
        new_xapriv_A=[];
        new_Papriv_A=[];
        new_yapriv=0;
        new_Yapriv=zeros(nx);
        new_uapriv=0;
        new_Uapriv=zeros(nz);
        new_Qapriv=Q0;
        new_Rapriv=R0;
        new_Lapriv=5;
        zA=[];

        for t=1:ts
            
            %%%%True noise covariance matrices
            Q=Q_s*(6.5+0.5*cos(0.5*2*pi*t/ts))*Q1;
            R=(0.1+0.05*cos(0.5*2*pi*t/ts))*R1;
            
            %%%%Extract data
            x=xA(:,t);
            z=ZA(:,t);
            
            %%%%Save measurement data
            if t<=(new_Lapriv+1)
                zA=[zA z];
            else
                zA=[zA(:,2:end) z];
            end
            
            %%%%Run VBAKF-PR and proposed SWVAKF
            [xapriv,Papriv,uapriv,Uapriv,Ppapriv,Rapriv,Qapriv]=aprivbkf(xapriv,Papriv,uapriv,Uapriv,F,H,z,Q0,N,tao_P,rou);
            
            [new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,new_Qapriv,new_Rapriv,new_Ppapriv]=...
            new_aprivbkf(new_xapriv,new_Papriv,new_xapriv_A,new_Papriv_A,new_yapriv,new_Yapriv,new_uapriv,new_Uapriv,F,H,zA,new_Qapriv,new_Rapriv,rou,new_Lapriv,t);

            %%%%MSE calculation
            mse_aprivbkf_1(i,t,expt)=(x(1)-xapriv(1))^2+(x(2)-xapriv(2))^2;
            mse_aprivbkf_2(i,t,expt)=(x(3)-xapriv(3))^2+(x(4)-xapriv(4))^2;
            
            mse_new_aprivbkf_1(i,t,expt)=(x(1)-new_xapriv(1))^2+(x(2)-new_xapriv(2))^2;
            mse_new_aprivbkf_2(i,t,expt)=(x(3)-new_xapriv(3))^2+(x(4)-new_xapriv(4))^2;
            
            %%%%Calculate the estimation error of Qk
            Q_aprivbkf(i,t,expt)=norm(Qapriv-Q,'fro')^2/nx^2;
            Q_new_aprivbkf(i,t,expt)=norm(new_Qapriv-Q,'fro')^2/nx^2;
            
            %%%%Calculate the estimation error of Rk
            R_aprivbkf(i,t,expt)=norm(Rapriv-R,'fro')^2/nz^2;
            R_new_aprivbkf(i,t,expt)=norm(new_Rapriv-R,'fro')^2/nz^2;

            %%%%NEES calculation
            nees_aprivbkf(i,t,expt)=(x-xapriv)'*inv(Papriv)*(x-xapriv);
            nees_new_aprivbkf(i,t,expt)=(x-new_xapriv)'*inv(new_Papriv)*(x-new_xapriv);
            
        end
        
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
%%%%%%%%%ARMSE calculation
rmse_aprivbkf_1=sqrt(mean(mse_aprivbkf_1,3));
rmse_aprivbkf_2=sqrt(mean(mse_aprivbkf_2,3));
rmse_new_aprivbkf_1=sqrt(mean(mse_new_aprivbkf_1,3));
rmse_new_aprivbkf_2=sqrt(mean(mse_new_aprivbkf_2,3));

%%%%%%%%%
armse_aprivbkf_1=mean(rmse_aprivbkf_1,2);
armse_aprivbkf_2=mean(rmse_aprivbkf_2,2);
armse_new_aprivbkf_1=mean(rmse_new_aprivbkf_1,2);
armse_new_aprivbkf_2=mean(rmse_new_aprivbkf_2,2);

%%%%%%%%%
L=length(armse_aprivbkf_1);

%%%%%%%%%ARMSE calculation
rmse_ktf_1=sqrt(mean(mse_ktf_1,3));
rmse_ktf_2=sqrt(mean(mse_ktf_2,3));
%%%%%%%%%
armse_ktf_1=mean(rmse_ktf_1);
armse_ktf_2=mean(rmse_ktf_2);
%%%%%%%%%
armse_ktf_1=repmat(armse_ktf_1,L,1);
armse_ktf_2=repmat(armse_ktf_2,L,1);

%%%%%%%ARMSE curves
figure;
subplot(2,1,1);
plot(alfa,armse_ktf_1,'-sk',alfa,armse_aprivbkf_1,'-*k',alfa,armse_new_aprivbkf_1,'-ok');
xlabel('Scale factor \alpha');
ylabel('ARMSE_{pos} (m)');
axis tight;
subplot(2,1,2);
plot(alfa,armse_ktf_2,'-sk',alfa,armse_aprivbkf_2,'-*k',alfa,armse_new_aprivbkf_2,'-ok');
xlabel('Scale factor \alpha');
ylabel('ARMSE_{vel} (m/s)');
axis tight;
legend('OKF','VBAKF-PR','Proposed SWVAKF');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%ANEES calculation
nees_aprivbkf=mean(nees_aprivbkf,3);
nees_new_aprivbkf=mean(nees_new_aprivbkf,3);
%%%%%%%%%
anees_aprivbkf=mean(nees_aprivbkf,2);
anees_new_aprivbkf=mean(nees_new_aprivbkf,2);

%%%%%%%%%ANEES calculation
nees_ktf=mean(nees_ktf,3);
anees_ktf=mean(nees_ktf);
anees_ktf=repmat(anees_ktf,L,1);

%%%%%%%ANEES curves
figure;
set(gcf,'unit','normalized','position',[0.2,0.2,0.41,0.3]);
plot(alfa,repmat(nx,L,1),'-b','linewidth',2.5);
hold on;
plot(alfa,anees_ktf,'-sk',alfa,anees_aprivbkf,'-*k',alfa,anees_new_aprivbkf,'-ok');
xlabel('Scale factor \alpha');
ylabel('ANEES');
legend('Reference value','OKF','VBAKF-PR','Proposed SWVAKF');
axis tight;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%Qk
NQ_aprivbkf=sqrt(sqrt(mean(Q_aprivbkf,3)));
NQ_new_aprivbkf=sqrt(sqrt(mean(Q_new_aprivbkf,3)));
%%%%%%%%%
ANQ_aprivbkf=mean(NQ_aprivbkf,2);
ANQ_new_aprivbkf=mean(NQ_new_aprivbkf,2);

%%%%%%%%%Rk
NR_aprivbkf=sqrt(sqrt(mean(R_aprivbkf,3)));
NR_new_aprivbkf=sqrt(sqrt(mean(R_new_aprivbkf,3)));
%%%%%%%%%
ANR_aprivbkf=mean(NR_aprivbkf,2);
ANR_new_aprivbkf=mean(NR_new_aprivbkf,2);

%%%%%%%AN curves
figure;
subplot(2,1,1);
plot(alfa,ANQ_aprivbkf,'-*k',alfa,ANQ_new_aprivbkf,'-ok');
xlabel('Scale factor \alpha');
ylabel('ASRNFN of \bf{Q}_{k}');
axis tight;
subplot(2,1,2);
plot(alfa,ANR_aprivbkf,'-*k',alfa,ANR_new_aprivbkf,'-ok');
xlabel('Scale factor \alpha');
ylabel('ASRNFN of \bf{R}_{k}');
axis tight;
legend('VBAKF-PR','Proposed SWVAKF','Location','SouthEast');