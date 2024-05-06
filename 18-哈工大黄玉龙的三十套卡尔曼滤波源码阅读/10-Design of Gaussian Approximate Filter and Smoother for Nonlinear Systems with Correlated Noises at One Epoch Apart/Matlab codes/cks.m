function [xcNA,PcNA]=cks(xcA,PcA,xck1kA,Pck1kA,Pckk1kA,T)

nx=size(xcA,1);

xck1N=xcA(:,T+1);

Pck1N=PcA(:,T*nx+1:T*nx+nx);

xcNA=xck1N;

PcNA=Pck1N;

for t=T-1:-1:0
    
    xkk=xcA(:,t+1);
    
    Pkk=PcA(:,t*nx+1:t*nx+nx);
    
    xk1k=xck1kA(:,t+1);
    
    Pk1k=Pck1kA(:,t*nx+1:t*nx+nx);
    
    Pkk1k=Pckk1kA(:,t*nx+1:t*nx+nx);
    
    Gk=Pkk1k*inv(Pk1k);
    
    xckN=xkk+Gk*(xck1N-xk1k);
    
    PckN=Pkk-Gk*(Pk1k-Pck1N)*Gk';

    xcNA=[xckN xcNA];
    
    PcNA=[PckN PcNA];
    
    xck1N=xckN;
    
    Pck1N=PckN;
    
end