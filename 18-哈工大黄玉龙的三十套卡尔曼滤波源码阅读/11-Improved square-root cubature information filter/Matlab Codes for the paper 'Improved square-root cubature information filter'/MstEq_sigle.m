function y=MstEq_sigle(x,xs,ys)

y1=sqrt((x(1)-xs)^2+(x(3)-ys)^2);

y2=atan2(x(3)-ys,x(1)-xs);

y=[y1;y2];