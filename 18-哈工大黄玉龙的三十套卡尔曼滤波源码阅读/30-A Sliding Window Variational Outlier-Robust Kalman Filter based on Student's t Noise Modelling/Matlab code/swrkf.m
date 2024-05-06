function [xkk,Pkk,xkk_A,Pkk_A,xkNA,PkNA,ykk_A,Ykk_A,ukk_A,Ukk_A,Q,R,ksi,lambda,i,M]=...
    swrkf(xkk_A,Pkk_A,ykk_A,Ykk_A,ukk_A,Ukk_A,Q,R,F,H,zA,ksi,lambda,omega,nu,rou,L,t,N,epsilon)
% Input:
% zA      measurements of time stamp k-M+1:k
% xkk_A   state estimates of time stamp k-M:k-1
% Output:
% xkk_A   state estimates of time stamp k-M+1:k
%%%%%
nx=size(xkk_A,1);
nz=size(zA,1);

if t<=(L)
    M=t;
    ksi=[ksi 1];%Time stamp k-M+1:k
    lambda=[lambda 1];
else
    M=L;
    ksi=[ksi(2:end) 1];%Time stamp k-M+1:k
    lambda=[lambda(2:end) 1];
end

yk1k=rou^M*ykk_A(1);
Yk1k=rou^M*Ykk_A(:,1:nx);
uk1k=rou^M*ukk_A(1);
Uk1k=rou^M*Ukk_A(:,1:nz);

xL=xkk_A(:,1);
PL=Pkk_A(:,1:nx);

xkk=zeros(nx,1);
for i=1:N
    %% x
    xkk_i=xkk;
    xkk=xL;
    Pkk=PL;
    xkA=xL;
    PkA=PL;
    for k=1:M       
        %%%%Time update
        xk1k=F*xkk;
        Qm{k}=Q/ksi(k);
        Pk1k=F*Pkk*F'+Qm{k};        
        %%%%Measurement update
        z=zA(:,k);        
        zk1k=H*xk1k;
        Rm{k}=R/lambda(k);
        Pzzk1k=H*Pk1k*H'+Rm{k};        
        Pxzk1k=Pk1k*H';        
        Kk=Pxzk1k*inv(Pzzk1k);        
        xkk=xk1k+Kk*(z-zk1k);        
        Pkk=Pk1k-Kk*H*Pk1k;
        Pkk=(Pkk+Pkk')/2;
        xkA=[xkA xkk];
        PkA=[PkA Pkk];
    end
    
    xkN=xkk;
    PkN=Pkk;
    xkNA=xkN;
    PkNA=PkN;        
    FQA=[];
    FRA=[];
    Ak=zeros(nx);
    Bk=zeros(nz);
    
    for j=M:(-1):1%M times smoothing
        %%%%%%Extracting the filtering estimate
        xk_1=xkA(:,j);
        Pk_1=PkA(:,(j-1)*nx+1:j*nx);
        
        %%%%%%Kalman smoothing
        xkk_1=F*xk_1;
        Pkk_1=F*Pk_1*F'+Qm{j};
        Gk_1=Pk_1*F'*inv(Pkk_1);
        xk_1N=xk_1+Gk_1*(xkN-xkk_1);
        Pk_1N=Pk_1+Gk_1*(PkN-Pkk_1)*Gk_1';
        
        %%%%%%Calculating the auxiliary variables
        Pk_1kN=Gk_1*PkN;
        F_Q=PkN-(F*Pk_1kN)'-F*Pk_1kN+F*Pk_1N*F'+(xkN-F*xk_1N)*(xkN-F*xk_1N)';
        FQA=[F_Q FQA];%time stamp k-M+1:k
        Ak=Ak+F_Q*ksi(j);
        
        %%%%%%Calculating the auxiliary variables
        z=zA(:,j);
        F_R=(z-H*xkN)*(z-H*xkN)'+H*PkN*H';
        FRA=[F_R FRA];%time stamp k-M+1:k
        Bk=Bk+F_R*lambda(j);
        
        %%%%%%
        xkN=xk_1N;
        PkN=Pk_1N;        
        xkNA=[xkN xkNA];
        PkNA=[PkN PkNA];
    end
    td=norm(xkk-xkk_i)/norm(xkk);
    
    if td<=epsilon && i>=2
        break;
    end
    
    %% ksi lambda
    alpha=zeros(1,M);
    beta=zeros(1,M);
    a=zeros(1,M);
    b=zeros(1,M);
    for j=1:M%time stamp k-M+1:k
        alpha(j)=0.5*(omega+nx);
        beta(j)=0.5*(omega+trace(FQA(:,(j-1)*nx+1:j*nx)*inv(Q)));
        a(j)=0.5*(nu+nz);
        b(j)=0.5*(nu+trace(FRA(:,(j-1)*nz+1:j*nz)*inv(R)));
    end
    ksi=alpha./beta;
    lambda=a./b;
    for j=1:M
        %%%%%%%preventing matrix singularities
        if ksi(j)<1e-16
            ksi(j)=1e-16;
        end
        %%%%%%%preventing matrix singularities
        if lambda(j)<1e-16
            lambda(j)=1e-16;
        end
    end
    %% Q R
    ykk=yk1k+M;
    Ykk=Yk1k+Ak;
    ukk=uk1k+M;
    Ukk=Uk1k+Bk;
    Q=Ykk/ykk;
    R=Ukk/ukk;
    Q=(Q+Q')/2;
    R=(R+R')/2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%Storing data
if t<(L)
    xkk_A=[xkk_A xkk];
    Pkk_A=[Pkk_A Pkk];
else
    xkk_A=[xkk_A(:,2:end) xkk];
    Pkk_A=[Pkk_A(:,(nx+1):end) Pkk];
end

if t<(L)
    ykk_A=[ykk_A ykk];
    Ykk_A=[Ykk_A Ykk];
    ukk_A=[ukk_A ukk];
    Ukk_A=[Ukk_A Ukk];
else
    ykk_A=[ykk_A(:,2:end) ykk];
    Ykk_A=[Ykk_A(:,(nx+1):end) Ykk];
    ukk_A=[ukk_A(:,2:end) ukk];
    Ukk_A=[Ukk_A(:,(nz+1):end) Ukk];
end