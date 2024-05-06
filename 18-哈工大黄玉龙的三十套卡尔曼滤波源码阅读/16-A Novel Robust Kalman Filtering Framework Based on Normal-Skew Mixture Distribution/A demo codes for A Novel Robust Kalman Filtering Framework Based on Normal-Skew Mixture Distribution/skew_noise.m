function y=skew_noise(Delta,R,nu)

n=size(R,1);

Lambda=zeros(n);

r1=20; 

for i=1:n
    
    Lambda(i,i)=gamrnd(nu/2,2/nu);  

end

inv_Lambda=inv(Lambda);

% u=abs(utchol(inv_Lambda)*randn(n,1));

u=abs(utchol(r1*inv_Lambda)*randn(n,1)); 

y=Delta*u+utchol(inv_Lambda*R)*randn(n,1);