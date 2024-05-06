function [xkk,Pkk,xkk_A,Pkk_A,ykk,Ykk,ukk,Ukk,Q,R,Pk1k]=new_aprivbkf(xkk,Pkk,xkk_A,Pkk_A,ykk,Ykk,ukk,Ukk,F,H,zA,Q,R,rou,L,t)

%%%%%
nx=size(xkk,1);

%%%%%%%%%%%%%%%%%%%%%%%%%Forward filtering
%%%%%Time update
xk1k=F*xkk;

Pk1k=F*Pkk*F'+Q;

%%%%%Measurement update
z=zA(:,end);

zk1k=H*xk1k;

Pzzk1k=H*Pk1k*H'+R;

Pxzk1k=Pk1k*H';

Kk=Pxzk1k*inv(Pzzk1k);

xkk=xk1k+Kk*(z-zk1k);

Pkk=Pk1k-Kk*H*Pk1k;

%%%%%%%%%%%%%%%%%%%%%%%%%
if t<=(L+1)
    xkk_A=[xkk_A xkk];
    Pkk_A=[Pkk_A Pkk];
else
    xkk_A=[xkk_A(:,2:end) xkk];
    Pkk_A=[Pkk_A(:,(nx+1):end) Pkk];
end

%%%%%%%%%%%%%%%%%%%%%%%%%Backward smoothing
if t>=2
    %%%%%%Prior information
    yk1k=rou*ykk;
    Yk1k=rou*Ykk;
    uk1k=rou*ukk;
    Uk1k=rou*Ukk;
    xkN=xkk;
    PkN=Pkk;
    
    %%%%%%
    Ak=zeros(nx);
    Bk=(z-H*xkN)*(z-H*xkN)'+H*PkN*H'; 
    
    %%%%%%
    if t<=(L+1)
        M=t-1;
    else
        M=L;
    end
    
    for j=M:-1:1
        %%%%%%
        xk_1=xkk_A(:,j);
        Pk_1=Pkk_A(:,(j-1)*nx+1:j*nx);
        
        %%%%%%
        xkk_1=F*xk_1;
        Pkk_1=F*Pk_1*F'+Q;
        Gk_1=Pk_1*F'*inv(Pkk_1);
        xk_1N=xk_1+Gk_1*(xkN-xkk_1);
        Pk_1N=Pk_1-Gk_1*(Pkk_1-PkN)*Gk_1';
        
        %%%%%%
        Pk_1kN=Gk_1*PkN;
        F_Q=PkN-(F*Pk_1kN)'-F*Pk_1kN+F*Pk_1N*F'+(xkN-F*xk_1N)*(xkN-F*xk_1N)';
        Ak=Ak+F_Q;
        
        %%%%%%
        z=zA(:,j);
        F_R=(z-H*xk_1N)*(z-H*xk_1N)'+H*Pk_1N*H'; 
        Bk=Bk+F_R;
        
        %%%%%%
        xkN=xk_1N;
        PkN=Pk_1N;
        
    end
    
    %%%%%%Update distribution parameters of Q
    ykk=yk1k+M;
    Ykk=Yk1k+Ak;

    %%%%%%Update estimate of Q
    Q=Ykk/ykk;
    
    %%%%%%Update distribution parameters of R
    ukk=uk1k+M+1;
    Ukk=Uk1k+Bk;
    
    %%%%%%Update estimate of R
    R=Ukk/ukk;
    
end
