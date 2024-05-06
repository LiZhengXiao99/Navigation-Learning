function Sigma2=match_1(u1,Sigma1,v1,v2,Nm)

n=size(u1,1);

%%%%%%
Sigma2=((v2-2)*v1)/((v1-2)*v2)*Sigma1;

for i=1:Nm
    
    Sigma20=Sigma2;
    
    sigma1=v1/(v1-2)*Sigma1;
    
    ci=((v1+trace(sigma1*inv(Sigma2)))/(v1+n))*((v1-2)/v1);
    
    Sigma2=ci*(v2+n)/(v2+trace(sigma1*inv(Sigma2)))*sigma1;
    
    if norm(Sigma2-Sigma20)<1e-98
        break;
    end
    
end

end

