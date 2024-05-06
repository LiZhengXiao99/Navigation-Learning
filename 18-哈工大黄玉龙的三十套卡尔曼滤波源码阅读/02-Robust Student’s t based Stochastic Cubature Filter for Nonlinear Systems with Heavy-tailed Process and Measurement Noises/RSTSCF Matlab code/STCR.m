function X=STCR(x,P,v)

nx = size(x,1);       

nPts = 2*nx;      

CPtArray = sqrt(nPts/2)*[eye(nx) -eye(nx)];

S = utchol(v/(v-2)*P);

X=repmat(x,1,nPts) + S*CPtArray;