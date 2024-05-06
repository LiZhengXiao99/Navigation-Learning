function y=ProssEq(x,T)

F=[1 sin(T*x(5))/x(5) 0 (-1+cos(T*x(5)))/x(5) 0;
    0 cos(T*x(5)) 0 -sin(T*x(5)) 0;
    0 (1-cos(T*x(5)))/x(5) 1 sin(T*x(5))/x(5) 0;
    0 sin(T*x(5)) 0 cos(T*x(5)) 0;
    0 0 0 0 1];

y=F*x;


