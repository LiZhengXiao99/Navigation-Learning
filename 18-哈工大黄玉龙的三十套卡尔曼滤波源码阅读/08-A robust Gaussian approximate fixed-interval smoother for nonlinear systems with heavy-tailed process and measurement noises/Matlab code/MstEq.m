function y=MstEq(x)

y=zeros(2,1);

y(1)=sqrt((x(1))^2+(x(2))^2);

y(2)=atan2(x(2),x(1));