function y=MstEq(x)

y1=atan2(x(3),x(1));

y2=atan2(x(3)+1000,x(1)+1000);

y3=atan2(x(3)-1000,x(1)+1000);

y4=atan2(x(3)+1000,x(1)-1000);

y5=atan2(x(3)-1000,x(1)-1000);

y=[y1;y2;y3;y4;y5];

