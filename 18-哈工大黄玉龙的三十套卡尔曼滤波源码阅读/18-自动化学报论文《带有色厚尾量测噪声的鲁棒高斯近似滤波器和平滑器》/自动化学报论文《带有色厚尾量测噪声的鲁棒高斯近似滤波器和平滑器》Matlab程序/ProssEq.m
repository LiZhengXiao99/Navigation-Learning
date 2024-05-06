function y=ProssEq(x)
% y为x的导数值
%x为系统的状态向量
%T时间间隔
T=1;
F=[1 sin(T*x(5))/x(5) 0 (-1+cos(T*x(5)))/x(5) 0;
    0 cos(T*x(5)) 0 -sin(T*x(5)) 0;
    0 (1-cos(T*x(5)))/x(5) 1 sin(T*x(5))/x(5) 0;
    0 sin(T*x(5)) 0 cos(T*x(5)) 0;
    0 0 0 0 1];
y=F*x;