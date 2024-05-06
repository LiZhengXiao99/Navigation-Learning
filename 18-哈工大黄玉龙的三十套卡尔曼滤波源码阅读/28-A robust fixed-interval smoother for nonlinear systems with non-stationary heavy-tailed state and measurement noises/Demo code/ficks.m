function [xkNA,PkNA,E_tao,E_pi]=ficks(xi,Pi,F,H,zA,ts,Q0,R0,e0,g0,N) 

%%%%Preparation
nx=size(xi,1);
nz=size(zA,1);

%%%%Initialization
v1=5;
v2=5;
h0=1-g0;
f0=1-e0;
%%%%%Process model
E_chi=ones(1,ts);
E_log_chi=zeros(1,ts);
E_t=ones(1,ts);
E_log_tao=(psi(g0)-psi(g0+h0))*ones(1,ts);
E_log_1_tao=(psi(h0)-psi(g0+h0))*ones(1,ts);

%%%%%Measurement model
E_lamda=ones(1,ts);
E_log_lamda=zeros(1,ts);
E_y=ones(1,ts);
E_log_pi=(psi(e0)-psi(e0+f0))*ones(1,ts);
E_log_1_pi=(psi(f0)-psi(e0+f0))*ones(1,ts);
%%%%%
xA_1=zeros(nx,ts+1);
epsilon=1e-8;   

for i=1:N

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

        %%%%%Calculate Q and R
        Q=Q0/(E_t(t)+(1-E_t(t))*E_chi(t));
        R=R0/(E_y(t)+(1-E_y(t))*E_lamda(t));
        
        %%%%%%Filtering
        [xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1]=ckf(xkk,Pkk,F,zA(:,t),Q,R);   
        
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
    KsA=[];  

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
        KsA=[Ks KsA];

    end
    
    %%%%Determine Convergence
    xA=xkNA;
    Sn=0;
    for j=1:(ts+1)
        Sn=Sn+(norm(xA(:,j)-xA_1(:,j)))^2;
    end
    Sn=sqrt(Sn/(ts+1));
    if Sn<=epsilon
%         fprintf('The number of final iteration of The propsoed KS = %d\n',i); 
%         return  
    else
%          fprintf('The number of running iteration of The propsoed KS = %d\n',i); 
    end
    %%%%%
    xA_1=xA;
    
    ek=e0;
    fk=1-e0;
    gk=g0;
    hk=1-g0;

    for t=1:ts
        
        %%%%%%%%%%%Extracte required parameters
        Gk_1=KsA(:,(t-1)*nx+1:t*nx);
        Pk_1N=PkNA(:,(t-1)*nx+1:t*nx);
        PkN=PkNA(:,t*nx+1:(t+1)*nx);
        xk_1N=xkNA(:,t);
        xkN=xkNA(:,t+1);
        z=zA(:,t);

        %%%%%%%%%%%Calculate auxiliary parameters
        Pk_1kN=Gk_1*PkN;
        
        %%%%%%%%%%%
        
        Xk_1Xk=CR([xk_1N;xkN],[Pk_1N Pk_1kN; Pk_1kN' PkN]);
        F_Q=(Xk_1Xk(nx+1:end,:)-ckf_ProssEq(Xk_1Xk(1:nx,:)))*(Xk_1Xk(nx+1:end,:)-ckf_ProssEq(Xk_1Xk(1:nx,:)))'/(2*nx);
        XkN=CR(xkN,PkN);
        F_R=(repmat(z,1,2*nx)-ckf_Mst(XkN))*(repmat(z,1,2*nx)-ckf_Mst(XkN))'/(2*nx);

        %%%%%%%%%%%Update the distribution parameters for \xi
        gama1=trace(F_Q*inv(Q0));
        eta_kk=0.5*nx*(1-E_t(t))+0.5*v1;
        theta_kk=0.5*gama1*(1-E_t(t))+0.5*v1;
        E_chi(t)=eta_kk/theta_kk;
        E_log_chi(t)=psi(eta_kk)-log(theta_kk);
        %%%%%%%%
        
        %%%%%%%%%%%Update the distribution parameters for \lambda
        gama2=trace(F_R*inv(R0));
        alfa_kk=0.5*nz*(1-E_y(t))+0.5*v2;
        beta_kk=0.5*gama2*(1-E_y(t))+0.5*v2;
        E_lamda(t)=alfa_kk/beta_kk;
        E_log_lamda(t)=psi(alfa_kk)-log(beta_kk);
        %%%%%%
        
       %%%%%%%Update mixing parameter y
       py1=exp(E_log_pi(t)-0.5*gama2);
       py0=exp(E_log_1_pi(t)+0.5*nz*E_log_lamda(t)-0.5*E_lamda(t)*gama2);
    
       if py1<=1e-98
           py1=py1+1e-98;
       end
    
       if py0<=1e-98
           py0=py0+1e-98;
       end
    
       E_y(t)=py1/(py1+py0);
       
       %%%%%%%Update mixing parameter t
       pt1=exp(E_log_tao(t)-0.5*gama1);
       pt0=exp(E_log_1_tao(t)+0.5*nx*E_log_chi(t)-0.5*E_chi(t)*gama1);
    
       if pt1<=1e-98
           pt1=pt1+1e-98;
       end
    
       if pt0<=1e-98
           pt0=pt0+1e-98;
       end
       
       E_t(t)=pt1/(pt1+pt0);
       
       %%%%%%%Update \pi
       ek=e0+E_y(t);
       fk=2-e0-E_y(t);

       %%%%%%%Update \tau
       gk=g0+E_t(t);
       hk=2-g0-E_t(t);
       
        E_log_pi(t)=psi(ek)-psi(ek+fk);
        E_log_1_pi(t)=psi(fk)-psi(ek+fk);

        E_log_tao(t)=psi(gk)-psi(gk+hk);
        E_log_1_tao(t)=psi(hk)-psi(gk+hk);

        E_pi(t)=ek/(ek+fk);
        E_tao(t)=gk/(gk+hk);

    end

end


