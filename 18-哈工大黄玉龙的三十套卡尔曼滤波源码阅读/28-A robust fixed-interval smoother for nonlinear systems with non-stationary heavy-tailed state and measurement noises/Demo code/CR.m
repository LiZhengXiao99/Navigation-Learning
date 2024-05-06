function X=CR(x,P)

S=utchol(P);

nx=size(x,1);                                   

nPts=2*nx;       

CPtArray=sqrt(nPts/2)*[eye(nx) -eye(nx)];

X=repmat(x,1,nPts)+S*CPtArray;

