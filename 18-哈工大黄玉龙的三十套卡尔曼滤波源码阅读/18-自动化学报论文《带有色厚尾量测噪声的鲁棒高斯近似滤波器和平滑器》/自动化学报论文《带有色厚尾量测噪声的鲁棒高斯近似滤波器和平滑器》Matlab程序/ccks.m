function [xkN,PkN]=ccks(xk1N,Pk1N,xk1k1,Pk1k1,xkk1,Pkk1,Ak)

%%%%%%%%%%ccks 表示有色平滑器
xkN=xkk1+Ak*(xk1N-xk1k1);

PkN=Pkk1+Ak*(Pk1N-Pk1k1)*Ak';











