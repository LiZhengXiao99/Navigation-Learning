function [xk1k1,Pk1k1,uk1,Uk1,ak1,bk1]=icckf(xkk,Pkk,y,Q,phi,uk,Uk,ak,bk,N)

%%%%%%%%%%%%%%%准备工作%%%%%%%%%%%%%%%%%
nx=size(xkk,1);
nz=size(y,1);
nPts0=2*nx;         %%%%%%%%%%%第一类Cubature point 数目
nPts1=2*(nx+nx);    %%%%%%%%%%%第二类Cubature point 数目

%%%%%%%%%%状态的一步预测%%%%%%%%%%%%%%%%
Xkk=CR(xkk,Pkk); 

Xk1k=ckf_ProssEq(Xkk);                  %%%%计算传播的容积点

xk1k=sum(Xk1k,2)/nPts0;

Pk1k=Xk1k*Xk1k'/nPts0-xk1k*xk1k'+Q;

Pxxkk1k=Xkk*Xk1k'/nPts0-xkk*xk1k';

xak1k=[xk1k;xkk];

Pak1k=[Pk1k Pxxkk1k';Pxxkk1k Pkk];

Xak1k=CR(xak1k,Pak1k);

Yk1k=ckf_Mst_new(Xak1k,phi);

yk1k=sum(Yk1k,2)/nPts1;

Sk=Yk1k*Yk1k'/nPts1-yk1k*yk1k';

Pxyk1k=Xak1k(1:nx,:)*Yk1k'/nPts1-xk1k*yk1k';

Pxykk1k=Xak1k(nx+1:end,:)*Yk1k'/nPts1-xkk*yk1k';

%%%%%%%%初始化处理
uk1=uk+1;
Uk1=Uk;
ak1=ak+0.5;
bk1=bk;
alfa=1;
beta=1;

%%%%%%%%计算初始的期望
E_lamda=alfa/beta;
E_inv_R=(uk1-nz-1)*inv(Uk1);
E_v=ak1/bk1;

for i=1:N
    
    %%%%%%%%%%计算R
    R=inv(E_inv_R)/E_lamda;

    %%%%%%%%%%%
    Pyyk1k=Sk+R;
    
    %%%%%%%%%一步平滑估计%%%%%%
    Kks=Pxykk1k*inv(Pyyk1k);

    xkk1=xkk+Kks*(y-yk1k);

    Pkk1=Pkk-Kks*Pyyk1k*Kks';
    
    %%%%%%%状态滤波估计%%%%%%%%%%%%
    Kk1=Pxyk1k*inv(Pyyk1k);     

    xk1k1=xk1k+Kk1*(y-yk1k);

    Pk1k1=Pk1k-Kk1*Pyyk1k*Kk1';

    %%%%%%%%计算互协方差矩阵
    Pxxkk1k1=Pxxkk1k-Kks*Pyyk1k*Kk1';   
    
    %%%%%%%%扩展状态估计
    xak1k1=[xk1k1;xkk1];
    Pak1k1=[Pk1k1 Pxxkk1k1';Pxxkk1k1 Pkk1];
    
    %%%%%%%%%%%%%%
    Xak1k1=CR(xak1k1,Pak1k1);
    Dk=(repmat(y,1,nPts1)-ckf_Mst_new(Xak1k1,phi))*(repmat(y,1,nPts1)-ckf_Mst_new(Xak1k1,phi))'/nPts1;
    
    %%%%%%%%%更新参数
    alfa=(nz+E_v)/2;
    beta=(E_v+trace(Dk*E_inv_R))/2;
    
    %%%%%%%%%%计算所需期望
    E_lamda=alfa/beta;
    E_log_lamda=psi(alfa)-log(beta);
    E_inv_R=(uk1-nz-1)*inv(Uk1);
    E_v=ak1/bk1;
    
    %%%%%%%%%%更新参数
    Uk1=Uk+E_lamda*Dk;
    bk1=bk-0.5-0.5*E_log_lamda+0.5*E_lamda;

end