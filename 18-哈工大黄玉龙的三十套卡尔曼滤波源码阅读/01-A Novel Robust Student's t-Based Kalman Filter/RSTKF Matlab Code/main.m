%%%%Set up
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;

%%%%Model parameter
nxp=1000;
nx=4;
nz=2;
T=1;        
q=1;
r=100;
F=[eye(2) T*eye(2);zeros(2) eye(2)];
H=[eye(2) zeros(2)];
Q0=[T^3/3*eye(2) T^2/2*eye(2);T^2/2*eye(2) T*eye(2)]*q;
R0=r*eye(2);
ts=100;

%%%%The choices of important parameters
U1=100;      
U2=100;      
p1=0.95;     
p2=0.90;     
N=10;        
v=5;         
tao_P=5;     

%%%%
D_Q=p1*Q0+(1-p1)*U1*Q0;
D_R=p2*R0+(1-p2)*U2*R0;

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%Initial values
    x=[0;0;0;0];                          
    P=diag([10000 10000 100 100]);        
    Skk=utchol(P);                        

    %%%%KF (Kalman filter)
    xf=x+Skk*randn(nx,1);                 
    Pf=P;

    %%%%The proposed ivbkf-fixed
    xiv=xf;
    Piv=Pf;

    %%%%The proposed ivbkf-\Sigma
    xapiv=xf;
    Papiv=Pf;
    
    %%%%Save data
    xA=x;
    xfA=xf;
    xivA=xiv;
    xapivA=xapiv;
    
    for t=1:ts
        
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

        [xiv,Piv]=ivbkf(xiv,Piv,F,H,z,Q0,R0,N,v,v);
        
        [xapiv,Papiv]=apivbkf(xapiv,Papiv,F,H,z,Q0,R0,N,v,v,tao_P);

        %%%%Save data
        xA=[xA x];
        xfA=[xfA xf];
        xivA=[xivA xiv];
        xapivA=[xapivA xapiv];
        
        %%%%MSE calculation
        mse_kf_1(1,t,expt)=(xA(1,t+1)-xfA(1,t+1))^2+(xA(2,t+1)-xfA(2,t+1))^2;
        mse_kf_2(1,t,expt)=(xA(3,t+1)-xfA(3,t+1))^2+(xA(4,t+1)-xfA(4,t+1))^2;
        
        mse_ivbkf_1(1,t,expt)=(xA(1,t+1)-xivA(1,t+1))^2+(xA(2,t+1)-xivA(2,t+1))^2;
        mse_ivbkf_2(1,t,expt)=(xA(3,t+1)-xivA(3,t+1))^2+(xA(4,t+1)-xivA(4,t+1))^2;
        
        mse_apivbkf_1(1,t,expt)=(xA(1,t+1)-xapivA(1,t+1))^2+(xA(2,t+1)-xapivA(2,t+1))^2;
        mse_apivbkf_2(1,t,expt)=(xA(3,t+1)-xapivA(3,t+1))^2+(xA(4,t+1)-xapivA(4,t+1))^2;

    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%%%%%%%%RMSE calculation
rmse_kf_1=sqrt(mean(mse_kf_1,3));
rmse_kf_2=sqrt(mean(mse_kf_2,3));

rmse_ivbkf_1=sqrt(mean(mse_ivbkf_1,3));
rmse_ivbkf_2=sqrt(mean(mse_ivbkf_2,3));

rmse_apivbkf_1=sqrt(mean(mse_apivbkf_1,3));
rmse_apivbkf_2=sqrt(mean(mse_apivbkf_2,3));

%%%%%%%RMSE curve
figure;
j = 1:ts;
plot(j*T,rmse_kf_1(1,:),'-k',j*T,rmse_ivbkf_1(1,:),'-b',j*T,rmse_apivbkf_1(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
legend('KFTCM','The proposed filter-fixed','The proposed filter-\Sigma');

figure;
j = 1:ts;
plot(j*T,rmse_kf_2(1,:),'-k',j*T,rmse_ivbkf_2(1,:),'-b',j*T,rmse_apivbkf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');
legend('KFTCM','The proposed filter-fixed','The proposed filter-\Sigma');
