function y=SKN(x,beta)

y=2*normpdf(x).*normcdf(beta*x);

end

