function [xkk,Skk]=New_ruckf(xkk,Skk,z,Q,R,N)

%%%%%%%%
nz=size(z,1);

nx=size(xkk,1);                                         

nPts=2*nx;         

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%%%%%Time update
xk1k1=xkk;

Sk1k1=utchol(Skk);

Xk1k1=repmat(xk1k1,1,nPts)+Sk1k1*CPtArray;

Xkk1=ckf_ProssEq(Xk1k1);                                

xkk1=sum(Xkk1,2)/nPts;       

Pkk1=Xkk1*Xkk1'/nPts-xkk1*xkk1'+Q;  

%%%%%%%%Measurement update
vkk1=zeros(nz,1);

Pvvkk1=R;

Pxvkk1=zeros(nx,nz);

for i=1:N
    
    %%%%%%%%
    Skk1=utchol(Pkk1);
    
    Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;
    
    Zi=ckf_Mst(Xi);
    
    %%%%%%%%
    hkk1=sum(Zi,2)/nPts;
    
    Phhkk1=Zi*Zi'/nPts-hkk1*hkk1';
    
    Pxhkk1=Xi*Zi'/nPts-xkk1*hkk1';
    
    zkk1=hkk1+vkk1;
    
    Hk=Pxhkk1'*inv(Pkk1);
    
    Pzzkk1=Phhkk1+Pvvkk1+Hk*Pxvkk1+(Hk*Pxvkk1)';
    
    Pxzkk1=Pxhkk1+Pxvkk1;
    
    Pvzkk1=(Hk*Pxvkk1)'+Pvvkk1;
    
    %%%%%%%%Recursive update
    rk=1/(N+1-i);
    
    Kk=rk*Pxzkk1*inv(Pzzkk1);  
    
    Wk=rk*Pvzkk1*inv(Pzzkk1);  
    
    xkk=xkk1+Kk*(z-zkk1);
    
    Pkk=Pkk1-Kk*Pzzkk1*Kk'; 
    
    vkk=vkk1+Wk*(z-zkk1);
    
    Pvvkk=Pvvkk1-Wk*Pzzkk1*Wk';   
    
    Pxvkk=Pxvkk1-Kk*Pzzkk1*Wk';
    
    %%%%%%%%Initial value fore next iteration
    xkk1=xkk;
    
    Pkk1=Pkk;
        
    vkk1=vkk;
    
    Pvvkk1=Pvvkk;
    
    Pxvkk1=Pxvkk;

end

xkk=xkk1;

Skk=Pkk1;

