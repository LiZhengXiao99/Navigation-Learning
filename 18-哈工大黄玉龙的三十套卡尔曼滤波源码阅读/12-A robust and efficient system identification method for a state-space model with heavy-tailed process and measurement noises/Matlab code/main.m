%%%%%%%%%%
clear all;
close all;
clc;
format long;
randn('state',sum(100*clock));     

%%%%%%%%%%
nxp=100;               % Number of Monte Carlo simulation or number of data realiztions
ts=1000;               % Step of time or number of samples
nx=1;                  % Dimension of state 
nz=1;                  % Dimension of measurement 
Ni=250;                % Number of iteration

%%%%%%%%%%True model and noise parameters
a=0.5;
b=2;
d=0.5;
%%%%%%%%%%
q=0.5;
t_w=1;
Q=1e-4;   
%%%%%%%%%%
r=0.5;
t_v=1;
R=1e-3;                

%%%%%%%%%%Save data for model parameters
I_aA=zeros(nxp,Ni+1);
I_bA=zeros(nxp,Ni+1);
I_dA=zeros(nxp,Ni+1);
%%%%%%%%%%Save data for process noise parameters
I_qA=zeros(nxp,Ni+1);
I_wA=zeros(nxp,Ni+1);
I_QA=zeros(nxp,Ni+1);
%%%%%%%%%%Save data for measurement noise parameters
I_rA=zeros(nxp,Ni+1);
I_vA=zeros(nxp,Ni+1);
I_RA=zeros(nxp,Ni+1);

for expt=1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%%%Initialization
    x=zeros(nx,1);                 % Initial true state value
    P=0.01;                        % Initial process state covariance
    SP=utchol(P);
    
    %%%%%%%%%%
    xc0=x+SP*randn(nx,1);          % Initial estimate
    Pc0=P;                         % Initial estimation error covariance matrix

    %%%%%%%%%%Save true state and measurement
    zA=[];
    xA=x;
    
    %%%%%%%%%%Simulate true state and measurement
    for t=1:ts 
        
        %%%%%%%%%%
        x=ProssEq(a,b,x,t)+q+utchol(Q)*trnd(t_w);
        z=MstEq(d,x)+r+utchol(R)*trnd(t_v);
        
        %%%%%%%%%%Save true state and measurement
        xA=[xA x];
        zA=[zA z];
        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%Initial values for gradient search
    I_a=a-0.1+0.2*rand;
    I_b=b-0.2+0.4*rand;
    I_d=d-0.1+0.2*rand;
    %%%%%%%%%%
    I_q=q-0.2+0.4*rand;
    I_w=t_w+4;
    I_Q=Q;
    %%%%%%%%%%
    I_r=r-0.2+0.4*rand;
    I_v=t_v+4;
    I_R=R;   

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%Save estimate results
    I_aA(expt,1)=I_a;
    I_bA(expt,1)=I_b;
    I_dA(expt,1)=I_d;
    %%%%%%%%%%
    I_qA(expt,1)=I_q;
    I_wA(expt,1)=I_w;
    I_QA(expt,1)=I_Q;
    %%%%%%%%%%
    I_rA(expt,1)=I_r;
    I_vA(expt,1)=I_v;
    I_RA(expt,1)=I_R;
    
    for i=1:Ni
        
        fprintf('Gradient search in Process = %d\n',i); 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Smoothing 
        [xcNA,PcNA,Pck_1kNA,E_chi,E_lamda,E_log_chi,E_log_lamda]=icks(xc0,Pc0,ts,zA,...
        I_a,I_b,I_d,I_q,I_w,I_Q,I_r,I_v,I_R,5);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [I_a,I_b,I_d,I_q,I_w,I_Q,I_r,I_v,I_R]=search_gradient(I_a,I_b,I_d,I_q,I_w,I_Q,I_r,I_v,I_R,...
        zA,xcNA,PcNA,Pck_1kNA,E_chi,E_lamda,E_log_chi,E_log_lamda);

        %%%%%%%%%%Save estimate results
        I_aA(expt,i+1)=I_a;
        I_bA(expt,i+1)=I_b;
        I_dA(expt,i+1)=I_d;
        %%%%%%%%%%
        I_qA(expt,i+1)=I_q;
        I_wA(expt,i+1)=I_w;
        I_QA(expt,i+1)=I_Q;
        %%%%%%%%%%
        I_rA(expt,i+1)=I_r;
        I_vA(expt,i+1)=I_v;
        I_RA(expt,i+1)=I_R;

    end

end

%%%%%%%%%%Estimation curves for model parameters
figure;
i=0:Ni;
subplot(3,1,1);
plot(i,repmat(a,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_aA(expt,:),'-k','LineWidth',0.005);
    hold on;
end
ylabel('parameter a');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,2);
plot(i,repmat(b,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_bA(expt,:),'-k','LineWidth',0.005);
    hold on;
end
ylabel('parameter b');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,3);
plot(i,repmat(d,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_dA(expt,:),'-k','LineWidth',0.005);
    hold on;
end
xlabel('Iteration number');
ylabel('parameter d');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)

%%%%%%%%%%Estimation curves for process noise parameters
figure;
i=0:Ni;
subplot(3,1,1);
plot(i,repmat(q,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_qA(expt,:),'-k','LineWidth',1);
    hold on;
end
ylabel('parameter q');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,2);
plot(i,repmat(t_w,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_wA(expt,:),'-k','LineWidth',1);
    hold on;
end
ylabel('parameter \omega');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,3);
plot(i,repmat(Q,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_QA(expt,:),'-k','LineWidth',0.005);
    hold on;
end
xlabel('Iteration number');
ylabel('parameter Q');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)

%%%%%%%%%%Estimation curves for measurement noise parameters
figure;
i=0:Ni;
subplot(3,1,1);
plot(i,repmat(r,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_rA(expt,:),'-k','LineWidth',1);
    hold on;
end
ylabel('parameter r');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,2);
plot(i,repmat(t_v,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_vA(expt,:),'-k','LineWidth',1);
    hold on;
end
ylabel('parameter \nu');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)
subplot(3,1,3);
plot(i,repmat(R,1,Ni+1),'-r','LineWidth',5);
hold on;
for expt=1:nxp
    plot(i,I_RA(expt,:),'-k','LineWidth',0.005);
    hold on;
end
xlabel('Iteration number');
ylabel('parameter R');
set(gca,'FontSize',12);
set(get(gca,'XLabel'),'FontSize',12)
set(get(gca,'YLabel'),'FontSize',12)