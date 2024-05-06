function X=STUT(x,P,v,kai)

nx=size(x,1);       

nPts=2*nx+1;      

CPtArray=sqrt(nx+kai)*[eye(nx) -eye(nx)];

S=utchol(v/(v-2)*P);

X=repmat(x,1,nPts)+[zeros(nx,1) S*CPtArray];