function [xkk,Skk]=ruekf(xkk,Skk,z,Q,R,N)

nx=size(xkk,1);        

nz=size(z,1);

%%%%%%%%Time update
F=d_ProssEq(xkk);    

xkk1=ProssEq(xkk); 

Pkk1=F*Skk*F'+Q;     

%%%%%%%%Measurement update
Pxvkk1=zeros(nx,nz);

for i=1:N
    
    H=d_MstEq(xkk1);  
    
    zkk1=MstEq(xkk1); 
    
    Pzzkk1=H*Pkk1*H'+R+H*Pxvkk1+(H*Pxvkk1)';  
    
    Pxzkk1=Pkk1*H'+Pxvkk1;   
    
    %%%%%%%%Recursive update
    rk=1/(N+1-i);
    
    Kk=rk*Pxzkk1*inv(Pzzkk1);  
    
    xkk=xkk1+Kk*(z-zkk1);
    
    Pkk=Pkk1-(2/rk-1)*Kk*Pzzkk1*Kk'; 
    
    Pxvkk=Pxvkk1-Kk*H*Pxvkk1-Kk*R;
    
    %%%%%%%%%%Initial value fore next iteration
    xkk1=xkk;
    
    Pkk1=Pkk;
    
    Pxvkk1=Pxvkk;
    
end

xkk=xkk1;

Skk=Pkk1;


