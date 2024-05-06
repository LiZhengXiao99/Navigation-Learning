function [xkNA,PkNA,Pk_1kNA,E_chi,E_lamda,E_log_chi,E_log_lamda]=icks(xi,Pi,ts,zA,a,b,d,q,w,Q,r,v,R,N)

%%%%
nx=size(xi,1);
nz=size(zA,1);

%%%%Initialization for expectations
E_chi=ones(1,ts);
E_lamda=ones(1,ts);
E_log_chi=zeros(1,ts);
E_log_lamda=zeros(1,ts);

for i=1:N

    %%%%Initial values for filtering
    xkk=xi;
    Pkk=Pi;
    %%%%Save data for filtering
    xkkA=xkk;
    PkkA=Pkk;
    xkk_1A=[];
    Pkk_1A=[];
    Pk_1kk_1A=[];
    
    for t=1:ts

        %%%%%Calculate modified Q and R
        M_Q=Q/E_chi(t);
        M_R=R/E_lamda(t);
        
        %%%%%%Filtering
        [xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1]=ckf(xkk,Pkk,t,zA(:,t),a,b,d,q,M_Q,r,M_R);   

        %%%%Save data
        xkkA=[xkkA xkk];
        PkkA=[PkkA Pkk];
        xkk_1A=[xkk_1A xkk_1];
        Pkk_1A=[Pkk_1A Pkk_1];
        Pk_1kk_1A=[Pk_1kk_1A Pk_1kk_1];

    end
    
    %%%%Initial values for smoothing
    xkN=xkk;
    PkN=Pkk;
    
    %%%%Save data for smoothing
    xkNA=xkN;
    PkNA=PkN; 
    KsA=[];  

    for t=(ts-1):-1:0
        
        %%%%%%Extract filtering estimates
        xkk=xkkA(:,t+1);
        Pkk=PkkA(:,t*nx+1:(t+1)*nx);
        xkk_1=xkk_1A(:,t+1);
        Pkk_1=Pkk_1A(:,t*nx+1:(t+1)*nx);
        Pk_1kk_1=Pk_1kk_1A(:,t*nx+1:(t+1)*nx);
        
        %%%%%%Smoothing
        [xkN,PkN,Ks]=cks(xkN,PkN,xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1);

        %%%%Save data
        xkNA=[xkN xkNA];
        PkNA=[PkN PkNA];
        KsA=[Ks KsA];

    end
    
    %%%%
    Pk_1kNA=[];
    %%%%Update parameters
    for t=1:ts
        
        %%%%%%%%%%%
        Gk_1=KsA(:,(t-1)*nx+1:t*nx);
        Pk_1N=PkNA(:,(t-1)*nx+1:t*nx);
        PkN=PkNA(:,t*nx+1:(t+1)*nx);
        xk_1N=xkNA(:,t);
        xkN=xkNA(:,t+1);
        z=zA(:,t);

        %%%%%%%%%%%
        Pk_1kN=Gk_1*PkN;
        Pk_1kNA=[Pk_1kNA Pk_1kN];
        
        %%%%%%%%%%%
        [F_Q,F_R]=FQR(xk_1N,xkN,Pk_1N,Pk_1kN,PkN,t,z,a,b,d,q,r);
        
        %%%%%%%%%%%Update \xi
        eta_kk=0.5*(nx+w);
        theta_kk=0.5*(w+trace(F_Q*inv(Q)));
        
        %%%%%%%%%%%Update \lambda
        alfa_kk=0.5*(nz+v);
        beta_kk=0.5*(v+trace(F_R*inv(R)));
        
        %%%%%%%%%%%Calculate E_chi and E_lamda
        E_chi(t)=eta_kk/theta_kk;
        E_lamda(t)=alfa_kk/beta_kk;
        
        %%%%%%%%%%%Calculate E_log_chi and E_log_lamda
        E_log_chi(t)=psi(eta_kk)-log(theta_kk);
        E_log_lamda(t)=psi(alfa_kk)-log(beta_kk);

    end
    
end


