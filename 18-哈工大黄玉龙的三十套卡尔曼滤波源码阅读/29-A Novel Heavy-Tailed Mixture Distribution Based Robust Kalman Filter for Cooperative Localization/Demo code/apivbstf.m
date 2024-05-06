function [xkk,Pkk,alphakk,ukk,Ukk,Etao,D_Pk1k,D_R]=apivbstf(xkk,Pkk,F,H,z,Q,R,alphakk,ukk,Ukk,pik,rou,M,N,v1,v2)

%%%%%%Set up
nz=size(z,1);

nx=size(xkk,1);

%%%%%Time update
xk1k=F*xkk;

for j=1:M
    Pk1k(nx*(j-1)+1:nx*j,:)=F*Pkk*F'+Q(nx*(j-1)+1:nx*j,:);
end

alphaklk=rou*alphakk;
uklk=rou*ukk;
Uklk=rou*Ukk;

for j=1:M
    tklk(j)=pik;
end

for j=1:M
    Tk1k(nx*(j-1)+1:nx*j,:)=pik*Pk1k(nx*(j-1)+1:nx*j,:);
end

xk_last=xk1k;

%%%%%Measurement update
ElogTao=psi(alphaklk)-psi(sum(alphaklk));

Elamada=alphaklk/sum(alphaklk);

E_Pk1k=zeros(nx);

for j=1:M
    E_Pk1k=E_Pk1k+Elamada(j)* Tk1k(nx*(j-1)+1:nx*j,:)/tklk(j); 
end 
E_i_Pk1k=inv(E_Pk1k);

E_i_Rk=ukk*inv(Ukk);

Egamma=1;

Eeta=1;

xkk=xk1k;

Pkk=E_Pk1k;

WTklk=zeros(nx,nx);

for i=1:N
    %%%%%%%Update the distributions of Pk1k 
    Ak=(xkk-xk1k)*(xkk-xk1k)'+Pkk;    
    
    Bk=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H'; 
    
    tkk=pik+1;
    
    for j=1:M
        WTklk=WTklk+Elamada(j)* Tk1k(nx*(j-1)+1:nx*j,:); 
    end 
    Tkk=WTklk+Ak*Egamma;   
    WTklk=zeros(nx,nx);
    
    E_i_Pk1k=tkk*inv(Tkk);
    
    %%%%%%%Update the distributions of Rk
    ukk=uklk+1;
    
    Ukk=Bk*Eeta+Uklk;
    
    E_i_Rk=ukk*inv(Ukk);
        
    %%%%%%%Update the distribution of state vector
    D_Pk1k=inv(E_i_Pk1k)/Egamma;

    D_R=inv(E_i_Rk)/Eeta;
    
    zk1k=H*xk1k;
    
    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%%Update the distributions of gamma and eta
    akk=0.5*(nx+v1);
    
    bkk=0.5*(trace(Ak*E_i_Pk1k)+v1);
    
    Egamma=akk/bkk;
    
    ckk=0.5*(nz+v2);
    
    dkk=0.5*(trace(Bk*E_i_Rk)+v2);
    
    Eeta=ckk/dkk;     
    
    %%%%%%%Update the distribution of lamada
    for j=1:M
       beta(j)=exp(0.5*pik*log(det(Tk1k(nx*(j-1)+1:nx*j,:)))-0.5*trace(Tk1k(nx*(j-1)+1:nx*j,:)*E_i_Pk1k)+ElogTao(j)); 
    end
    
    betakk=beta/sum(beta);
    
    Elamada=betakk;
    
    %%%%%%%Update the distribution of tao
    alphakk=Elamada+alphaklk;
    
    Etao=alphakk/sum(alphakk);
    
    ElogTao=psi(alphakk)-psi(sum(alphakk));
    
    %%%%%%%Break
    dist=norm(xkk-xk_last)/norm(xkk);
    xk_last=xkk;
    
    if dist<1e-17
        break;
    end
    
end



