function [F_R]=FR(xk_1N,xkN,Pk_1N,Pk_1kN,PkN,y,phi)

xakN=[xkN;xk_1N];

PakN=[PkN Pk_1kN';Pk_1kN Pk_1N];

XakN=CR(xakN,PakN);

L=size(XakN,2);

F_R=(repmat(y,1,L)-ckf_Mst_new(XakN,phi))*(repmat(y,1,L)-ckf_Mst_new(XakN,phi))'/L;

