function [fx]=funcs(gama,n,sigma,v,flag)

%%%%%%%第一种选择
if flag==1
    fx = sigma*sigma*exp(0.5*(n-gama)/sigma^2);
end

%%%%%%%第二种选择
if flag==2
    fx = -0.5*(v+n)*log(n + gama/v);
end

%%%%%%%第三种选择
if flag==3
    fx = -sqrt((v+n)*(v+gama));
end


end

