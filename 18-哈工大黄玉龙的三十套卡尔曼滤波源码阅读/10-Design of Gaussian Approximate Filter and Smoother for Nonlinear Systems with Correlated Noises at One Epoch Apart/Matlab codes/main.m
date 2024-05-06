%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));

%%%%%%Model parameters
raddeg=pi/180;
nx=5;         
nz=2;         
nxp=1000;     
ts=100;       
T=1;          
Nk=ts/T;      
M=[T^3/3 T^2/2;T^2/2 T];

%%%%%%Noise parameters
q1=0.1;
q2=1.75e-4;
Cr=10;
Co=0.1*sqrt(10)*1e-3;  

%%%%%%Noise covariance matrices
Q=[q1*M zeros(2,2) zeros(2,1);zeros(2,2) q1*M zeros(2,1);zeros(1,2) zeros(1,2) q2*T];
R=diag([Cr^2 Co^2]);
S=[0.50 0.00;
   0.50 0.00;
   0.20 0.00;
   0.20 0.00;
   0.10 0.00];
QR=[Q S;S' R];
SQR=(chol(QR))';

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    %%%%%Initialization
    x=[1000;300;1000;0;-3*raddeg];     
    P=diag([100 10 100 10 1e-4]); 
    Skk=utchol(P);                     
    
    %%%%CKF
    xc=x+Skk*randn(nx,1);
    Pc=P;
    
    %%%%Correlated-CKF
    c_xc=xc;
    c_Pc=Pc;
    
    %%%%Save data for filters
    xA=x;
    xcA=xc;
    c_xcA=c_xc;
    %%%%%%%%%%%%%%%%%%%%%%%%Correlated-CKS
    c_PcA=c_Pc;
    zA=[];
    axcA=[];
    aPcA=[];
    KA=[];
    K1A=[];
    %%%%%%%%%%%%%%%%%%%%%%%%Standard CKS
    PcA=Pc;
    xck1kA=[];
    Pck1kA=[];
    Pckk1kA=[];
    
    for t=1:Nk
        
        wv=SQR*randn(nx+nz,1);
        
        w=wv(1:nx);
        
        v=wv(nx+1:nx+nz);
        
        x=ProssEq(x)+w;
        
        z=MstEq(x)+v;
        
        %%%%%%%%%%%%%%%Filtering
        [xc,Pc,xck1k,Pck1k,Pckk1k]=ckf(xc,Pc,z,Q,R);
        
        [c_xc,c_Pc,axc,aPc,K,K1]=correlated_ckf(c_xc,c_Pc,z,Q,R,S);
        
        xA=[xA x];
   
        xcA=[xcA xc];
        
        c_xcA=[c_xcA c_xc];
        
        %%%%%%%%%%%%%%%Save data for Standard CKS
        PcA=[PcA Pc];
        
        xck1kA=[xck1kA xck1k];
        
        Pck1kA=[Pck1kA Pck1k];
        
        Pckk1kA=[Pckk1kA Pckk1k];

        %%%%%%%%%%%%%%%Save data for correlated-CKS
        zA=[zA z];
        
        c_PcA=[c_PcA c_Pc];
        
        axcA=[axcA axc];
        
        aPcA=[aPcA aPc];
        
        KA=[KA K];
        
        K1A=[K1A K1];
        
    end
    
    [xcNA,PcNA]=cks(xcA,PcA,xck1kA,Pck1kA,Pckk1kA,Nk);
    
    [c_xcNA,c_PcNA]=correlated_cks(c_xcA,c_PcA,zA,axcA,aPcA,KA,K1A,Nk);
    
    abs_ckf_1(expt,:)=(xA(1,:)-xcA(1,:)).^2+(xA(3,:)-xcA(3,:)).^2;
    abs_ckf_2(expt,:)=(xA(2,:)-xcA(2,:)).^2+(xA(4,:)-xcA(4,:)).^2;
    abs_ckf_3(expt,:)=(xA(5,:)-xcA(5,:)).^2;
    
    
    abs_c_ckf_1(expt,:)=(xA(1,:)-c_xcA(1,:)).^2+(xA(3,:)-c_xcA(3,:)).^2;
    abs_c_ckf_2(expt,:)=(xA(2,:)-c_xcA(2,:)).^2+(xA(4,:)-c_xcA(4,:)).^2;
    abs_c_ckf_3(expt,:)=(xA(5,:)-c_xcA(5,:)).^2;
    
    
    abs_cks_1(expt,:)=(xA(1,:)-xcNA(1,:)).^2+(xA(3,:)-xcNA(3,:)).^2;
    abs_cks_2(expt,:)=(xA(2,:)-xcNA(2,:)).^2+(xA(4,:)-xcNA(4,:)).^2;
    abs_cks_3(expt,:)=(xA(5,:)-xcNA(5,:)).^2;
    
    
    abs_c_cks_1(expt,:)=(xA(1,:)-c_xcNA(1,:)).^2+(xA(3,:)-c_xcNA(3,:)).^2;
    abs_c_cks_2(expt,:)=(xA(2,:)-c_xcNA(2,:)).^2+(xA(4,:)-c_xcNA(4,:)).^2;
    abs_c_cks_3(expt,:)=(xA(5,:)-c_xcNA(5,:)).^2;
        
end

abs_ckf_1=sqrt(mean(abs_ckf_1,1));
abs_ckf_2=sqrt(mean(abs_ckf_2,1));
abs_ckf_3=sqrt(mean(abs_ckf_3,1));

abs_c_ckf_1=sqrt(mean(abs_c_ckf_1,1));
abs_c_ckf_2=sqrt(mean(abs_c_ckf_2,1));
abs_c_ckf_3=sqrt(mean(abs_c_ckf_3,1));

abs_cks_1=sqrt(mean(abs_cks_1,1));
abs_cks_2=sqrt(mean(abs_cks_2,1));
abs_cks_3=sqrt(mean(abs_cks_3,1));
        
abs_c_cks_1=sqrt(mean(abs_c_cks_1,1));
abs_c_cks_2=sqrt(mean(abs_c_cks_2,1));
abs_c_cks_3=sqrt(mean(abs_c_cks_3,1));        

    
figure;
j = 0:Nk;
plot(j,abs_ckf_1(1,:),'-k',j,abs_cks_1(1,:),'-b',j,abs_c_ckf_1(1,:),'-r',j,abs_c_cks_1(1,:),'-g','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE-pos(m)');
legend('Existing CKF','Existing CKS','The proposed CKF','The proposed CKS');

figure;
j = 0:Nk;
plot(j,abs_ckf_2(1,:),'-k',j,abs_cks_2(1,:),'-b',j,abs_c_ckf_2(1,:),'-r',j,abs_c_cks_2(1,:),'-g','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE-vel(m/s)');
legend('Existing CKF','Existing CKS','The proposed CKF','The proposed CKS');


figure;
j = 0:Nk;
plot(j,abs_ckf_3(1,:)./raddeg,'-k',j,abs_cks_3(1,:)./raddeg,'-b',j,abs_c_ckf_3(1,:)./raddeg,'-r',j,abs_c_cks_3(1,:)./raddeg,'-g','linewidth',2.5);
xlabel('Time(s)');
ylabel('RMSE-ome(Deg/s)');
legend('Existing CKF','Existing CKS','The proposed CKF','The proposed CKS');
      