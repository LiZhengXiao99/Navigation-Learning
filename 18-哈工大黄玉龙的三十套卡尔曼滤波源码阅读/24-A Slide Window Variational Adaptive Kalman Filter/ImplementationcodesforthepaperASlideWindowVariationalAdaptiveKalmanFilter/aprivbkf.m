function [xkk,Pkk,ukk,Ukk,D_Pk1k,D_R,D_Q]=aprivbkf(xkk,Pkk,ukk,Ukk,F,H,z,Q,N,tao1,rou)

%%%%%
Pk1k1=Pkk;

%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

tk1k=tao1;

Tk1k=tao1*Pk1k;

uk1k=rou*ukk;

Uk1k=rou*Ukk;

%%%%%Measurement update
xkk=xk1k;

Pkk=Pk1k;

for i=1:N
    
    %%%%%%%
    xkki=xkk;
    
    %%%%%%%Update IW distribution parameters for Pk1k
    Ak=(xkk-xk1k)*(xkk-xk1k)'+Pkk;     
    
    tkk=tk1k+1;                                     
    
    Tkk=Tk1k+Ak;                                  
    
    E_i_Pk1k=tkk*inv(Tkk);                     

    %%%%%%%Update IW distribution parameters for Rk
    Bk=(z-H*xkk)*(z-H*xkk)'+H*Pkk*H';  
    
    ukk=uk1k+1;                        
    
    Ukk=Uk1k+Bk;                      
    
    E_i_Rk=ukk*inv(Ukk);           

    %%%%%%%Update state estimate
    D_Pk1k=inv(E_i_Pk1k);

    D_R=inv(E_i_Rk);
    
    zk1k=H*xk1k;

    Pzzk1k=H*D_Pk1k*H'+D_R;
    
    Pxzk1k=D_Pk1k*H';
    
    Kk=Pxzk1k*inv(Pzzk1k);
    
    xkk=xk1k+Kk*(z-zk1k);
    
    Pkk=D_Pk1k-Kk*H*D_Pk1k;
    
    %%%%%%    
    delta=norm(xkk-xkki)/norm(xkk);
    
    if delta<=1e-8
        break;        
    end
    
end

D_Q=D_Pk1k-F*Pk1k1*F';