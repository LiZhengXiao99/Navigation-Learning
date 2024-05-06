function [F_Q,F_R]=FQR(xk_1N,xkN,Pk_1N,Pk_1kN,PkN,t,z,a,b,d,q,r)

nx=size(xk_1N,1);

%%%%%%%%%%Calculate F_Q   4*nx
ckN=[xk_1N;xkN];

PcckN=[Pk_1N Pk_1kN;Pk_1kN' PkN];

CkN=CR(ckN,PcckN);

Delta_w=CkN(nx+1:2*nx,:)-ckf_ProssEq(a,b,CkN(1:nx,:),t)-repmat(q,1,4*nx);

F_Q=Delta_w*Delta_w'/(4*nx);

%%%%%%%%%%Calculate F_R   2*nx
XkN=CR(xkN,PkN);

Delta_v=repmat(z,1,2*nx)-ckf_Mst(d,XkN)-repmat(r,1,2*nx);

F_R=Delta_v*Delta_v'/(2*nx);






