function [c_xcNA,c_PcNA]=correlated_cks(c_xcA,c_PcA,zA,axcA,aPcA,KA,K1A,T)

nx=size(c_xcA,1);

nz=size(zA,1);

c_xck1N=c_xcA(:,T+1);

c_Pck1N=c_PcA(:,T*nx+1:T*nx+nx);

c_xcNA=c_xck1N;

c_PcNA=c_Pck1N;

for t=T-1:-1:0
    
    xkk=c_xcA(:,t+1);
    
    Pkk=c_PcA(:,t*nx+1:t*nx+nx);
    
    axc=axcA(:,t+1);
    
    ax=[c_xck1N;zA(:,t+1)];
    
    aPc=aPcA(:,t*(nx+nz)+1:t*(nx+nz)+(nx+nz));
    
    K=KA(:,t*(nx+nz)+1:t*(nx+nz)+(nx+nz));
    
    K1=K1A(:,t*nx+1:t*nx+nx);
    
    c_xckN=xkk+K*(ax-axc);

    c_PckN=Pkk-K*aPc*K'+K1*c_Pck1N*K1';
    
    c_xcNA=[c_xckN c_xcNA];
    
    c_PcNA=[c_PckN c_PcNA];
    
    c_xck1N=c_xckN;
    
    c_Pck1N=c_PckN;
    
end