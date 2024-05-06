function [xkNA,PkNA]=mcks(xi,Pi,zA,ts,Q,R0,v,N)

%%%%初始处理
nx=size(xi,1);
nz=size(zA,1);
E_lamda=ones(1,ts);

for i=1:N

    %%%%滤波初值
    xkk=xi;
    Pkk=Pi;
    %%%%滤波存储
    xkkA=xkk;
    PkkA=Pkk;
    xkk_1A=[];
    Pkk_1A=[];
    Pk_1kk_1A=[]; 
    
    for t=1:ts
        
        %%%%%计算R
        R=R0/E_lamda(t);
        
        %%%%%%调用滤波程序
        [xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1]=ckf(xkk,Pkk,zA(:,t),Q,R);   
        
        %%%%滤波存储
        xkkA=[xkkA xkk];
        PkkA=[PkkA Pkk];
        xkk_1A=[xkk_1A xkk_1];
        Pkk_1A=[Pkk_1A Pkk_1];
        Pk_1kk_1A=[Pk_1kk_1A Pk_1kk_1];

    end
    
    %%%%平滑初值
    xkN=xkk;
    PkN=Pkk;
    
    %%%%平滑存储
    xkNA=xkN;
    PkNA=PkN;

    for t=(ts-1):-1:0
        
        %%%%%%提取滤波估计
        xkk=xkkA(:,t+1);
        Pkk=PkkA(:,t*nx+1:(t+1)*nx);
        xkk_1=xkk_1A(:,t+1);
        Pkk_1=Pkk_1A(:,t*nx+1:(t+1)*nx);
        Pk_1kk_1=Pk_1kk_1A(:,t*nx+1:(t+1)*nx);
        
        %%%%%%调用平滑程序
        [xkN,PkN,Ks]=cks(xkN,PkN,xkk,Pkk,xkk_1,Pkk_1,Pk_1kk_1);
        
        %%%%平滑存储
        xkNA=[xkN xkNA];
        PkNA=[PkN PkNA];

    end
    
    for t=1:ts
        
        %%%%%%%%%%%提取所需参数
        xkN=xkNA(:,t+1);
        PkN=PkNA(:,t*nx+1:(t+1)*nx);
        z=zA(:,t);
        
        %%%%%%%%%%%计算辅助量
        XkN=CR(xkN,PkN);
        F_R=(repmat(z,1,2*nx)-ckf_Mst(XkN))*(repmat(z,1,2*nx)-ckf_Mst(XkN))'/(2*nx);
        
        %%%%%%%%%%%计算lamda的均值
        gama=trace(F_R*inv(R0));
        E_lamda(t)=(v+nz)/(v+gama);

    end

end
