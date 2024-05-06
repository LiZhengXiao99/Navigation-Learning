function Pu=fun_gk_i(c,sigma)

n=size(c,1);

sigma2=sigma^2;

pu=zeros(n,1);

for i=1:n
    
    ci=c(i);

    pu(i)=exp(0.5/sigma2*(-ci^2));

    if pu(i)<1e-8
        pu(i)=pu(i)+1e-8;
    end

end

Pu=diag(pu);