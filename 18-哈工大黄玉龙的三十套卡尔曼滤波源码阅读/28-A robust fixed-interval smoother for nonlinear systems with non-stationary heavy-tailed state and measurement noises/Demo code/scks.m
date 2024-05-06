function [xkNA,PkNA]=scks(xi,Pi,F,zA,ts,QA,RA)

%%%%Preparation
nx=size(xi,1);
nz=size(zA,1);

%%%%Initial value for filter
xkk=xi;
Pkk=Pi;

%%%%Save data for filter
xkkA=xkk;
PkkA=Pkk;
xkk_1A=[];
Pkk_1A=[];
Pk_1kk_1A=[];

for t=1:ts
        
    %%%%%%Filtering
    [xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1]=ckf(xkk,Pkk,F,zA(:,t),QA(:,nx*(t-1)+1:nx*t),RA(:,nz*(t-1)+1:nz*t));   
        
    %%%%Save data for filter
    xkkA=[xkkA xkk];
    PkkA=[PkkA Pkk];
    xkk_1A=[xkk_1A xkk_1];
    Pkk_1A=[Pkk_1A Pkk_1];
    Pk_1kk_1A=[Pk_1kk_1A Pk_1kk_1];

end

%%%%Initial value for smoother
xkN=xkk;
PkN=Pkk;
    
%%%%Save data for smoother
xkNA=xkN;
PkNA=PkN;

for t=(ts-1):-1:0
        
    %%%%%%Extracte filtering estimate
    xkk=xkkA(:,t+1);
    Pkk=PkkA(:,t*nx+1:(t+1)*nx);
    xkk_1=xkk_1A(:,t+1);
    Pkk_1=Pkk_1A(:,t*nx+1:(t+1)*nx);
    Pk_1kk_1=Pk_1kk_1A(:,t*nx+1:(t+1)*nx);
        
    %%%%%%Smoothing
    [xkN,PkN,Ks]=cks(xkN,PkN,xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1);

    %%%%Save data for smoother
    xkNA=[xkN xkNA];
    PkNA=[PkN PkNA];

end

