%%%%
clear all;
close all;
clc;

%%%%Set range for random variable
delta=0.02;
x=-5:delta:10;     
mx=x(2:end);

%%%%Parameter selections
u=0;
sigma=1;
nu=3;
beta=2;
M=1e4;

%%%%Skew normal distribution
y_SKN=SKN(x,beta);  
dy_SKN=C_IF(y_SKN,delta);
fprintf('SKN ok!\n');

%%%%Skew t distribution
y_SKT=SKT(x,nu,beta);  
dy_SKT=C_IF(y_SKT,delta);
fprintf('SKT ok!\n');

%%%%SGGM distribution
y_SGGM=SGGM(x,u,sigma,nu,beta,M);  
dy_SGGM=C_IF(y_SGGM,delta);
fprintf('SGGM ok!\n');

%%%%SGEM distribution
y_SGEM=SGEM(x,u,sigma,nu,beta,M);      
dy_SGEM=C_IF(y_SGEM,delta);
fprintf('SGEM ok!\n');

%%%%SGBM distribution
y_SGBM=SGBM(x,u,sigma,nu,beta,M);    
dy_SGBM=C_IF(y_SGBM,delta);
fprintf('SGBM ok!\n');

%%%%SGIGM·Ö²¼
y_SGIGM=SGIGM(x,u,sigma,nu,beta,M);    
dy_SGIGM=C_IF(y_SGIGM,delta);
fprintf('SGIGM ok!\n');

%%%%SGIEM distribution
y_SGIEM=SGIEM(x,u,sigma,nu,beta,M);    
dy_SGIEM=C_IF(y_SGIEM,delta);
fprintf('SGIEM ok!\n');

%%%%Plot PDF figure
figure;
plot(x,y_SKN,'-k',x,y_SKT,'--k',x,y_SGGM,'-m',x,y_SGEM,'-c','linewidth',2.5);
hold on;
plot(x,y_SGBM,'-g',x,y_SGIGM,'-b',x,y_SGIEM,'-r','linewidth',2.5);
xlabel('Random variable \rm x');
ylabel('Probability density');
legend('Skew normal','Skew-t','SGGM','SGEM','SGBM','SGIGM','SGIEM');
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);

%%%%Plot log-PDF figure
figure;
plot(x,log(y_SKN),'-k',x,log(y_SKT),'--k',x,log(y_SGGM),'-m',x,log(y_SGEM),'-c','linewidth',2.5);
hold on;
plot(x,log(y_SGBM),'-g',x,log(y_SGIGM),'-b',x,log(y_SGIEM),'-r','linewidth',2.5);
xlabel('Random variable \rm x');
ylabel('Logarithm of probability density');
legend('Skew normal','Skew-t','SGGM','SGEM','SGBM','SGIGM','SGIEM');
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);

%%%%Plot influence function figure
figure;
plot(mx,dy_SKN,'-k',mx,dy_SKT,'--k',mx,dy_SGGM,'-m',mx,dy_SGEM,'-c','linewidth',2.5);
hold on;
plot(mx,dy_SGBM,'-g',mx,dy_SGIGM,'-b',mx,dy_SGIEM,'-r','linewidth',2.5);
xlabel('Random variable \rm x');
ylabel('Influence function');
legend('Skew normal','Skew-t','SGGM','SGEM','SGBM','SGIGM','SGIEM');
set(gca,'FontSize',15);
set(get(gca,'XLabel'),'FontSize',15);
set(get(gca,'YLabel'),'FontSize',15);