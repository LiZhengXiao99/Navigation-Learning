function y=f_g_b_1(a,b,q,Q,xt,xt1,t)

y0=(1/Q)*(xt/(1+xt^2));

y1=xt1-ProssEq(a,b,xt,t)-q;

y=y0*y1;
