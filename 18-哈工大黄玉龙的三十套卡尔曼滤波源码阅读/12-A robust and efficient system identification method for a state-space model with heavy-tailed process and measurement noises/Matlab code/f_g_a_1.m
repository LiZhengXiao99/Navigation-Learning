function y=f_g_a_1(a,b,q,Q,xt,xt1,t)

y0=(1/Q)*xt;

y1=xt1-ProssEq(a,b,xt,t)-q;

y=y0*y1;