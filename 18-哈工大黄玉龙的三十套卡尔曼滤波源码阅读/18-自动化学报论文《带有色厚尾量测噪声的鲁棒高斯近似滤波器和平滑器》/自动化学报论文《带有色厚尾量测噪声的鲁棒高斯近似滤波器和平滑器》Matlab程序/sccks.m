function [xkNA,PkNA]=sccks(xs,Ps,yA,Q,phi,ts,R)

%%%%初始处理
nx=size(xs,1);

%%%%滤波初值
xkk=xs;
Pkk=Ps;

%%%%滤波存储
xkkA=xkk;
PkkA=Pkk;
xk_1kA=[];
Pk_1kA=[];
KsA=[];

for t=1:ts
    
    y=yA(:,t);
        
    %%%%%%调用滤波程序
    [xkk,Pkk,xk_1k,Pk_1k,Ks] = cckf(xkk,Pkk,y,Q,R,phi);

    %%%%滤波存储
    xkkA=[xkkA xkk];
    PkkA=[PkkA Pkk];
    xk_1kA=[xk_1kA xk_1k];
    Pk_1kA=[Pk_1kA Pk_1k];
    KsA=[KsA Ks];

end

%%%%平滑初值
xkN=xkk;
PkN=Pkk;
    
%%%%平滑存储
xkNA=xkN;
PkNA=PkN;

for t=(ts-1):-1:0
        
    %%%%%%提取滤波估计
    xkk=xkkA(:,t+2);
    Pkk=PkkA(:,(t+1)*nx+1:(t+2)*nx);
    xkk1=xk_1kA(:,t+1);
    Pkk1=Pk_1kA(:,t*nx+1:(t+1)*nx);
    Ak=KsA(:,t*nx+1:(t+1)*nx);

    %%%%%%调用平滑程序
    [xkN,PkN]=ccks(xkN,PkN,xkk,Pkk,xkk1,Pkk1,Ak);

    %%%%平滑存储
    xkNA=[xkN xkNA];
    PkNA=[PkN PkNA];

end
