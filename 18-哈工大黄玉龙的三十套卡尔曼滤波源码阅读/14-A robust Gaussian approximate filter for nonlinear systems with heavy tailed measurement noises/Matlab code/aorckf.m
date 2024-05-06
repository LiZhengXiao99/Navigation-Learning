function [xkk,Skk,akk,bkk,ukk,Ukk]=aorckf(xk1k1,Sk1k1,z,Q,ak1k1,bk1k1,uk1k1,Uk1k1,rou,N)

%%%%
nx=size(xk1k1,1);    

nz=size(z,1);

nPts=2*nx;     

%%%%Time update
%%%%State prediction
Xk1k1=CR(xk1k1,Sk1k1);
    
Xkk1=ckf_ProssEq(Xk1k1);              

xkk1=sum(Xkk1,2)/nPts;       
                  
Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;     

%%%%Parameter prediction
ukk1=rou*(uk1k1-nz-1)+nz+1;

Ukk1=rou*Uk1k1;

akk1=rou*ak1k1;

bkk1=rou*bk1k1;

%%%%Measurement update
%%%%Initialization
ukk=ukk1+1;

akk=akk1+0.5;

Ukk=Ukk1;

bkk=bkk1;

%%%%Initialize expectations for the parameters
E_lamda=1;

E_IR=(ukk-nz-1)*inv(Ukk);

E_v=akk/bkk;

for i=1:N
    
    %%%%Update state
    R=inv(E_IR)/E_lamda;   %%%%Calculate modified measurement noise covariance matrix
    
    Xi=CR(xkk1,Pkk1);
    
    Zi=ckf_Mst(Xi);
    
    zkk1=sum(Zi,2)/nPts;  

    Pzzkk1=Zi*Zi'/nPts-zkk1*zkk1'+R;
    
    Pxzkk1=Xi*Zi'/nPts-xkk1*zkk1';

    Wk=Pxzkk1*inv(Pzzkk1);      

    xkk = xkk1 + Wk*(z - zkk1);
    
    Skk=Pkk1-Wk*Pzzkk1*Wk';   
    
    %%%%Calculate the auxiliary parameter E[(y-h(x))(y-h(x))']
    Xkk=CR(xkk,Skk);
    
    Zkk=ckf_Mst(Xkk);
    
    Zkk=repmat(z,1,nPts)-Zkk;
    
    Dk=Zkk*Zkk'/nPts;
    
    %%%%Update lamda
    vk1=(nz+E_v)/2;
    
    vk2=(trace(Dk*E_IR)+E_v)/2;
    
    E_lamda=vk1/vk2;
    
    E_ln_lamda=psi(vk1)-log(vk2);
    
    %%%%Update R
    Ukk=Ukk1+E_lamda*Dk;           
    
    E_IR=(ukk-nz-1)*inv(Ukk);
    
    %%%%Update v
    bkk=bkk1-0.5-0.5*E_ln_lamda+0.5*E_lamda;   
    
    E_v=akk/bkk;

end

