function [xkk,Skk] = kf(xkk,Skk,F,H,z,Q,R)

%%%%%%%%Time update
xkk1=F*xkk;       

Pkk1=F*Skk*F'+Q;             
    
zkk1 = H*xkk1;           

Pzzkk1=H*Pkk1*H'+R;

Pxzkk1=Pkk1*H';

%%%%%%%%Measurement update
Wk=Pxzkk1*inv(Pzzkk1);            
    
xkk = xkk1 + Wk*(z - zkk1);

Skk=Pkk1-Wk*Pzzkk1*Wk';   