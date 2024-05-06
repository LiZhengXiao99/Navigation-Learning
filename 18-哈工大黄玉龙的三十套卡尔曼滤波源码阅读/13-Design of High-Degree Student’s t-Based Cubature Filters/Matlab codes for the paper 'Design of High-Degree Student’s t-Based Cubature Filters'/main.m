%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nxp=1000;                          %%%Monte Carlo simulation number
raddeg=pi/180;                     %%%deg to rad
nx=5;                              
nz=5;                              
ts=50;                             %%%Runing time
T=1.0;                             
Nk=ts/T;                           
M=[T^3/3 T^2/2;T^2/2 T];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Model parameters
q1=0.08*0.1;
q2=0.20*1.75e-4;
Q0=[q1*M zeros(2,2) zeros(2,1);zeros(2,2) q1*M zeros(2,1);zeros(1,2) zeros(1,2) q2*T]; 
R0=(0.08*raddeg)^2*eye(nz);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1=0.90;
p2=0.95;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%dof parameters%%%%%%%%%%%
c_v1=5;                             %%%dof parameter of state noise
c_v2=c_v1;                          %%%dof parameter of measurement noise
c_v3=c_v1;                          %%%dof parameter of posterior PDF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%%%%%%%%%%%%initial values
    x=[10000;300;10000;100;-6*raddeg];    
    P=diag([100 10 100 10 1e-5]);         
    Skk=utchol(P);                        

    %%%%CKF
    xc=x+Skk*randn(nx,1);                 
    Pc=P;
    
    %%%%STUF
    st_xu=xc;
    st_Pu=Pc;

    %%%%STCKF
    st_xc=xc;
    st_Pc=Pc;

    %%%%Genz?5th?degree STCF
    sth_xc=xc;
    sth_Pc=Pc;
    
    %%%%Mysovskikh?5th?degree STCF
    sthssr_xc=xc;
    sthssr_Pc=Pc;
    
    %%%%Save data
    xA=x;
    st_xuA=st_xu;
    st_xcA=st_xc;
    sth_xcA=sth_xc;
    sthssr_xcA=sthssr_xc;
    
    for t=1:Nk
        
        %%%%%%%%%%%%%%Simulate heavy-tailed noises
        test1=rand;
        test2=rand;
        
        if test1<=p1
            Q=Q0;
        else
            Q=100*Q0;
        end
       %%%%%%%%%%%%%%
        if test2<=p2
            R=R0;
        else
            R=50*R0;
        end
        
        SR=utchol(R);    
        SQ=utchol(Q);    
        
        %%%%%%%%%%%%%%Simulate true state and measurement
        x=ProssEq(x)+SQ*randn(nx,1); 
        z=MstEq(x)+SR*randn(nz,1);

        %%%%%%%%%%%%%%Filtering
        [st_xu,st_Pu] = stukf(st_xu,st_Pu,z,Q0,R0,c_v1,c_v2,c_v3,3-nx);

        [st_xc,st_Pc]=stckf(st_xc,st_Pc,z,Q0,R0,c_v1,c_v2,c_v3);
        
        [sth_xc,sth_Pc]=sthckf(sth_xc,sth_Pc,z,Q0,R0,c_v1,c_v2,c_v3);

        [sthssr_xc,sthssr_Pc]=sthssrckf(sthssr_xc,sthssr_Pc,z,Q0,R0,c_v1,c_v2,c_v3);

        %%%%%%%%%%%%%%Save data
        xA=[xA x];  
        st_xuA=[st_xuA st_xu];
        st_xcA=[st_xcA st_xc];
        sth_xcA=[sth_xcA sth_xc];
        sthssr_xcA=[sthssr_xcA sthssr_xc];
        
        %%%%%%%%%%%%%%¼ÆËãMSE
        abs_st_ukf_1(1,t,expt)=(xA(1,t)-st_xuA(1,t))^2+(xA(3,t)-st_xuA(3,t))^2;
        abs_st_ukf_2(1,t,expt)=(xA(2,t)-st_xuA(2,t))^2+(xA(4,t)-st_xuA(4,t))^2;
        abs_st_ukf_3(1,t,expt)=(xA(5,t)-st_xuA(5,t))^2;
        
        abs_st_ckf_1(1,t,expt)=(xA(1,t)-st_xcA(1,t))^2+(xA(3,t)-st_xcA(3,t))^2;
        abs_st_ckf_2(1,t,expt)=(xA(2,t)-st_xcA(2,t))^2+(xA(4,t)-st_xcA(4,t))^2;
        abs_st_ckf_3(1,t,expt)=(xA(5,t)-st_xcA(5,t))^2;
        
        abs_sth_ckf_1(1,t,expt)=(xA(1,t)-sth_xcA(1,t))^2+(xA(3,t)-sth_xcA(3,t))^2;
        abs_sth_ckf_2(1,t,expt)=(xA(2,t)-sth_xcA(2,t))^2+(xA(4,t)-sth_xcA(4,t))^2;
        abs_sth_ckf_3(1,t,expt)=(xA(5,t)-sth_xcA(5,t))^2;
        
        abs_sthssr_ckf_1(1,t,expt)=(xA(1,t)-sthssr_xcA(1,t))^2+(xA(3,t)-sthssr_xcA(3,t))^2;
        abs_sthssr_ckf_2(1,t,expt)=(xA(2,t)-sthssr_xcA(2,t))^2+(xA(4,t)-sthssr_xcA(4,t))^2;
        abs_sthssr_ckf_3(1,t,expt)=(xA(5,t)-sthssr_xcA(5,t))^2;

    end

end

%%%%%%%Calculate RMSE     
abs_st_ukf_1=sqrt(mean(abs_st_ukf_1,3));
abs_st_ukf_2=sqrt(mean(abs_st_ukf_2,3));
abs_st_ukf_3=sqrt(mean(abs_st_ukf_3,3));

abs_st_ckf_1=sqrt(mean(abs_st_ckf_1,3));
abs_st_ckf_2=sqrt(mean(abs_st_ckf_2,3));
abs_st_ckf_3=sqrt(mean(abs_st_ckf_3,3));

abs_sth_ckf_1=sqrt(mean(abs_sth_ckf_1,3));
abs_sth_ckf_2=sqrt(mean(abs_sth_ckf_2,3));
abs_sth_ckf_3=sqrt(mean(abs_sth_ckf_3,3));
     
abs_sthssr_ckf_1=sqrt(mean(abs_sthssr_ckf_1,3));
abs_sthssr_ckf_2=sqrt(mean(abs_sthssr_ckf_2,3));
abs_sthssr_ckf_3=sqrt(mean(abs_sthssr_ckf_3,3));
     
%%%%%%%%%%%%%%RMSE figure
figure;
j=T*(1:Nk);
plot(j,abs_st_ukf_1(1,:),'-g',j,abs_st_ckf_1(1,:),'-k',j,abs_sth_ckf_1(1,:),'-b',j,abs_sthssr_ckf_1(1,:),'-r','linewidth',2.5);
ylabel('RMSE_{pos} (m)');
legend('STUF','3rd-degree STCF','Genz-5th-degree STCF','Mysovskikh-5th-degree STCF'); 
xlabel('Time (s)');
ylabel('RMSE_{pos} (m)');

figure;
j=T*(1:Nk);
plot(j,abs_st_ukf_2(1,:),'-g',j,abs_st_ckf_2(1,:),'-k',j,abs_sth_ckf_2(1,:),'-b',j,abs_sthssr_ckf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{vel} (m/s)');

figure;
j=T*(1:Nk);
plot(j,abs_st_ukf_3(1,:)./raddeg,'-g',j,abs_st_ckf_3(1,:)./raddeg,'-k',j,abs_sth_ckf_3(1,:)./raddeg,'-b',j,abs_sthssr_ckf_3(1,:)./raddeg,'-r','linewidth',2.5);
xlabel('Time (s)');
ylabel('RMSE_{ome} (Deg/s)');


%%%%%%%%%%%%%%Calculate ARMSE
armse_st_ukf_1=mean(abs_st_ukf_1)
armse_st_ukf_2=mean(abs_st_ukf_2)
armse_st_ukf_3=mean(abs_st_ukf_3)/raddeg

armse_st_ckf_1=mean(abs_st_ckf_1)
armse_st_ckf_2=mean(abs_st_ckf_2)
armse_st_ckf_3=mean(abs_st_ckf_3)/raddeg

armse_sth_ckf_1=mean(abs_sth_ckf_1)
armse_sth_ckf_2=mean(abs_sth_ckf_2)
armse_sth_ckf_3=mean(abs_sth_ckf_3)/raddeg

armse_sthssr_ckf_1=mean(abs_sthssr_ckf_1)
armse_sthssr_ckf_2=mean(abs_sthssr_ckf_2)
armse_sthssr_ckf_3=mean(abs_sthssr_ckf_3)/raddeg

