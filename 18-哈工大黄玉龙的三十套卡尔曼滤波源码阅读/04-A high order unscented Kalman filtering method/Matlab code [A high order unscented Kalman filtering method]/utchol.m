function C = utchol(P)

[n,m] = size(P);
if (n-m) error('non-square argument'); end;
  for j=m:-1:1,
    for i=j:-1:1,
    sigma = P(i,j);
      for k=j+1:m,
      sigma = sigma - C(i,k)*C(j,k);
      end;
    C(j,i) = 0;
      if (i==j)
        C(i,j) = sqrt(max([0,sigma]));
      elseif (C(j,j) == 0)
        C(i,j) = 0;
      else
        C(i,j) = sigma/C(j,j);
      end;
    end;
  end;
      