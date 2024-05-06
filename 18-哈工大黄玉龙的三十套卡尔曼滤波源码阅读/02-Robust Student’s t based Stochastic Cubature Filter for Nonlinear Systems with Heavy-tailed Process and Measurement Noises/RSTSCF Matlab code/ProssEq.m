function y=ProssEq(x)

T=1;

F=[eye(2) T*eye(2);zeros(2) eye(2)];

y=F*x;