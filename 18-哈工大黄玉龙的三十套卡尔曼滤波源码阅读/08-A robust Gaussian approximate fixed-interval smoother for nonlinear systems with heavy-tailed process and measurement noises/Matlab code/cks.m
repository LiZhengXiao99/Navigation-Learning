function [xkN,SkN,Gk]=cks(xkN,SkN,xkk,Skk,xk1k,Pk1k,Pkk1k)

xk1N=xkN;

Sk1N=SkN;

Gk=Pkk1k*inv(Pk1k);

xkN=xkk+Gk*(xk1N-xk1k);

SkN=Skk-Gk*(Pk1k-Sk1N)*Gk';


