%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
randn('state',sum(100*clock));     
format long;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%Model parameter%%%%%%%%%
raddeg=pi/180;                     %%%deg to rad
velkn=1.852/60;                    %%%km/min
nx=4;                              
nz=1;                              
nxp=1000;                          %%%The number of Monte Carlo run
ts=100;                            %%%Run time
T=1;                               %%%Sampling interval
G=[0.5*T^2 0;T 0;0 0.5*T^2;0 T];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%
vp=50*velkn;                                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%
xp=0;                              
yp=0;                              
for t=1:ts
    
    if t<=15
        yaw_1=-80*raddeg;
    else
        yaw_1=146*raddeg;
    end
    
    xp=xp+vp*sin(yaw_1)*t;
    
    yp=yp+vp*cos(yaw_1)*t;
    
    xpA(t)=xp;
    
    ypA(t)=yp;
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%noise variance%%%%%%%%%%%
Q_0=0.001^2*eye(2);
Q0=G*Q_0*G';                        
R0=0.02^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p1=0.95;
p2=0.95;

%%%%dof parameter%%%%%%%%%%%%
c_v1=5;                             
c_v2=c_v1;                          
c_v3=c_v1;                          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for expt = 1:nxp
    
    fprintf('MC Run in Process = %d\n',expt); 
    
    %%%%%initial value%%%%%%%%%
    yaw_2=-135.4*raddeg;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    vt=180*velkn;                                               
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    vtx=vt*sin(yaw_2);
    vty=vt*cos(yaw_2);
    x=[3.0;3.0;vtx;vty];              
    P=diag([16 16 4 4]);              
    Skk=utchol(P);                    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%CKF
    xc=x+Skk*randn(nx,1);                 
    Pc=P;
  
    %%%%STEKF
    st_xe=xc;
    st_Pe=Pc;
    
    %%%%STUF
    st_xu=xc;
    st_Pu=Pc;

    %%%%STCKF
    st_xc=xc;
    st_Pc=Pc;

    %%%%STHCKF
    sth_xc=xc;
    sth_Pc=Pc;

    %%%%RSTSCF
    stsi_xc=xc;
    stsi_Pc=Pc;
    
    %%%%Save data
    xA=x;
    st_xeA=st_xe;
    st_xuA=st_xu;
    st_xcA=st_xc;
    sth_xcA=sth_xc;
    stsi_xcA=stsi_xc;
    
    for t=1:ts
        
        %%%%%%%%%%%%%%Produce heavy-tailed process and measurement noises
        test1=rand;
        test2=rand;
        
        if test1<=p1
            vx=G*utchol(Q_0)*randn(2,1);
        else
            vx=G*utchol(100*Q_0)*randn(2,1);
        end
       %%%%%%%%%%%%%%
        if test2<=p2
            vz=utchol(R0)*randn(nz,1);
        else
            vz=utchol(50*R0)*randn(nz,1);
        end
        
        %%%%%%%%%%%%%%
        xp=xpA(t);
        yp=ypA(t);
        
        %%%%%%%%%%%%%%Simulate true state and measurement
        x=ProssEq(x)+vx;
        
        z=MstEq(x,xp,yp)+vz;

        %%%%%%%%%%%%%%Filtering
        [st_xe,st_Pe]=stekf(st_xe,st_Pe,z,Q0,R0,c_v3,xp,yp);

        [st_xu,st_Pu]=stukf(st_xu,st_Pu,z,Q0,R0,c_v1,c_v2,c_v3,3-nx,xp,yp);
        
        [st_xc,st_Pc]=stckf(st_xc,st_Pc,z,Q0,R0,c_v1,c_v2,c_v3,xp,yp);
        
        [sth_xc,sth_Pc]=sthckf(sth_xc,sth_Pc,z,Q0,R0,c_v1,c_v2,c_v3,xp,yp);

        [stsi_xc,stsi_Pc]=stsickf(stsi_xc,stsi_Pc,z,Q0,R0,c_v1,c_v2,c_v3,100,xp,yp);
        
        %%%%%%%%%%%%%%Save data
        xA=[xA x];  
        st_xeA=[st_xeA st_xe];
        st_xuA=[st_xuA st_xu];
        st_xcA=[st_xcA st_xc];
        sth_xcA=[sth_xcA sth_xc];
        stsi_xcA=[stsi_xcA stsi_xc];
        
        %%%%%%%%%%%%%%MSE
        abs_st_ekf_1(1,t,expt)=(xA(1,t)-st_xeA(1,t))^2+(xA(2,t)-st_xeA(2,t))^2;
        abs_st_ekf_2(1,t,expt)=(xA(3,t)-st_xeA(3,t))^2+(xA(4,t)-st_xeA(4,t))^2;
        
        abs_st_ukf_1(1,t,expt)=(xA(1,t)-st_xuA(1,t))^2+(xA(2,t)-st_xuA(2,t))^2;
        abs_st_ukf_2(1,t,expt)=(xA(3,t)-st_xuA(3,t))^2+(xA(4,t)-st_xuA(4,t))^2;
        
        abs_st_ckf_1(1,t,expt)=(xA(1,t)-st_xcA(1,t))^2+(xA(2,t)-st_xcA(2,t))^2;
        abs_st_ckf_2(1,t,expt)=(xA(3,t)-st_xcA(3,t))^2+(xA(4,t)-st_xcA(4,t))^2;
        
        abs_sth_ckf_1(1,t,expt)=(xA(1,t)-sth_xcA(1,t))^2+(xA(2,t)-sth_xcA(2,t))^2;
        abs_sth_ckf_2(1,t,expt)=(xA(3,t)-sth_xcA(3,t))^2+(xA(4,t)-sth_xcA(4,t))^2;

        abs_stsi_ckf_1(1,t,expt)=(xA(1,t)-stsi_xcA(1,t))^2+(xA(2,t)-stsi_xcA(2,t))^2;
        abs_stsi_ckf_2(1,t,expt)=(xA(3,t)-stsi_xcA(3,t))^2+(xA(4,t)-stsi_xcA(4,t))^2;

    end

end

%%%%%%%RMSE     
abs_st_ekf_1=sqrt(mean(abs_st_ekf_1,3));
abs_st_ekf_2=sqrt(mean(abs_st_ekf_2,3));

abs_st_ukf_1=sqrt(mean(abs_st_ukf_1,3));
abs_st_ukf_2=sqrt(mean(abs_st_ukf_2,3));
 
abs_st_ckf_1=sqrt(mean(abs_st_ckf_1,3));
abs_st_ckf_2=sqrt(mean(abs_st_ckf_2,3));
 
abs_sth_ckf_1=sqrt(mean(abs_sth_ckf_1,3));
abs_sth_ckf_2=sqrt(mean(abs_sth_ckf_2,3));

abs_stsi_ckf_1=sqrt(mean(abs_stsi_ckf_1,3));
abs_stsi_ckf_2=sqrt(mean(abs_stsi_ckf_2,3));

%%%%%%%%%%%%%%RMSE curves
figure;
j=T*(1:ts);
plot(j,abs_st_ekf_1(1,:),'--g',j,abs_st_ukf_1(1,:),'-g',j,abs_st_ckf_1(1,:),'--m',j,abs_sth_ckf_1(1,:),'-m','linewidth',2.5);
hold on;
plot(j,abs_stsi_ckf_1(1,:),'-r','linewidth',2.5);
xlabel('Time (min)');
ylabel('RMSE_{pos} (km)');
legend('RSTEF','3rd-degree RSTUF','3rd-degree RSTCF','5th-degree RSTUF','RSTSCF'); 

figure;
j=T*(1:ts);
plot(j,abs_st_ekf_2(1,:),'--g',j,abs_st_ukf_2(1,:),'-g',j,abs_st_ckf_2(1,:),'--m',j,abs_sth_ckf_2(1,:),'-m','linewidth',2.5);
hold on;
plot(j,abs_stsi_ckf_2(1,:),'-r','linewidth',2.5);
xlabel('Time (min)');
ylabel('RMSE_{vel} (km/min)');
legend('RSTEF','3rd-degree RSTUF','3rd-degree RSTCF','5th-degree RSTUF','RSTSCF'); 
