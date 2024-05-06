%%%%Set up
clear all;
close all;
clc;
randn('state',sum(100*clock));
format long;

%%%%Model parameter
nxp=1000;
nx=1;
nz=1;
F=0.5;
H=1;
Q=100;
R0=100;
ts=100;
T=0.01;
Nk=ts/T;

%%%%The choices of important parameters
U2=100;
p2=0.90;
Nm=50;
v=5;
tao_P=5;

%%%%
D_R=p2*R0+(1-p2)*U2*R0;

for expt=1:nxp
    
    fprintf('MC Run in Process = %d\n',expt);
    
    %%%%Initial values
    x=100;
    P=1000;
    Skk=utchol(P);
    xf=x+Skk*randn(nx,1);
    
    %%%%RSTKF
    xapiv=xf;
    Papiv=P;
    akk=5;
    Akk=akk*R0;
    
    %%%%The proposed filter 
    xiv=xf;
    Piv=P;
    ukk=0;
    Ukk=100;
    tkk=5;
    Tkk=tkk*R0;
    rou=1-exp(-4);
    
    %%%%Save data
    xA=x;
    zA=[];
    xapivA=xapiv;
    xivA=xiv;
    ukkA=ukk;
    
    for t=1:Nk
        
        %%%%The true bias
        if t<Nk/3
            u=10;
        elseif t>2*Nk/3
            u=10;
        else
            u=30;
        end
        
        test2=rand;
        
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
        z=H*x+SR*randn(nz,1)+u;
        
        %%%%Filtering       
        [xapiv,Papiv,akk,Akk]=arvbkf(xapiv,Papiv,F,H,z,Q,akk,Akk,Nm,v,rou);
        
        [xiv,Piv,ukk,Ukk,tkk,Tkk]=ivbkf(xiv,Piv,F,H,z,Q,ukk,Ukk,tkk,Tkk,Nm,v,rou);
        
        %%%%Save data
        xA=[xA x];
        xapivA=[xapivA xapiv];
        xivA=[xivA xiv];
        ut(t)=u;  
        ukkA=[ukkA ukk];
        
        %%%%MSE calculation 
        mse_apivbkf(1,t,expt)=(xA(1,t+1)-xapivA(1,t+1))^2;
        
        mse_ivbkf(1,t,expt)=(xA(1,t+1)-xivA(1,t+1))^2;
        
        mse_ukk(1,t,expt)=ukkA(1,t+1);
        
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%RMSE calculation
rmse_apivbkf=sqrt(mean(mse_apivbkf,3));

rmse_ivbkf=sqrt(mean(mse_ivbkf,3));

rmse_ukk=mean(mse_ukk,3);

%%%%%%%RMSE curve
figure;
j=1:Nk;
plot(j*T,rmse_apivbkf(1,:),'-b',j*T,rmse_ivbkf(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');
legend('ORSTKF','Proposed filter');

figure;
j=1:Nk;
plot(j*T,ut,'-b',j*T,rmse_ukk(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
legend('True bias','Estimated bias');

