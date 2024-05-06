function y=MstEq(x)
%%%%%测量方程为离散的，不需要离散化
%x为系统的状态向量

y=zeros(2,1);

y(1)=sqrt((x(1))^2+(x(3))^2);

y(2)=atan2(x(3),x(1));
