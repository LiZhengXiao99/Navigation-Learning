function X=STHSSR(x,P,v)

S=utchol(v/(v-4)*P);

nx=size(x,1);     

nPts=nx^2+3*nx+3; 

%%%%%% sigma point sampling %%%%%%%%%%%%%%
sigma1=zeros(nx,nx+1);

sigma2=[];

for j=1:(nx+1)
    
    for i=1:nx
        if(i<j)
            sigma1(i,j)=-sqrt((nx+1)/(nx*(nx-i+2)*(nx-i+1)));
        end
        
        if(i==j)
            sigma1(i,j)=sqrt((nx+1)*(nx-i+1)/(nx*(nx-i+2)));
        end
        
        if(i>j)
            sigma1(i,j)=0;
        end
     
    end
    
end

for l=2:nx+1
    
    for k=1:l-1
        
        sigma2=[sigma2 sqrt(nx/(2*(nx-1)))*(sigma1(:,k)+sigma1(:,l))];  
        
    end
      
end

t_sigma1=sqrt(nx+2)*S*sigma1;

t_sigma2=sqrt(nx+2)*S*sigma2;

X=repmat(x,1,nPts)+[zeros(nx,1),t_sigma1,-t_sigma1,t_sigma2,-t_sigma2];

