function [Xi]=genetate_CPt(xkk1,Pkk1)

%%%%%%%%%%
nx=size(xkk1,1);                                         

nPts=2*nx;    

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%%%%%%%%Measurement update
Skk1=utchol(Pkk1);

Xi=repmat(xkk1,1,nPts)+Skk1*CPtArray;