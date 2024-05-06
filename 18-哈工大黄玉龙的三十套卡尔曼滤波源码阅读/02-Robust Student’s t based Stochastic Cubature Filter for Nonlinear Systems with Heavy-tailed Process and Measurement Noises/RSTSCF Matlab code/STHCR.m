function X=STHCR(x,P,v)

S = utchol(v/(v-4)*P);

nx=size(x,1);

nPts = 2*nx^2+1; 

CPtArray=eye(nx);

sj1=[];

sj2=[];

for k=1:(nx-1)
    
    for m=(k+1):nx
        
        e1=sqrt(0.5)*(CPtArray(:,k)+CPtArray(:,m));
        
        e2=sqrt(0.5)*(CPtArray(:,k)-CPtArray(:,m));
        
        sj1=[sj1,e1];
        
        sj2=[sj2,e2];
        
    end
    
end

sj3=eye(nx);

sigma1=sqrt(nx+2)*S*sj1;

sigma2=sqrt(nx+2)*S*sj2;

sigma3=sqrt(nx+2)*S*sj3;

X=repmat(x,1,nPts)+[zeros(nx,1),sigma1,-sigma1,sigma2,-sigma2,sigma3,-sigma3];

