function x=ProssEq(a,b,x,t)

c=8;

x=a*x+b*x/(1+x^2)+c*cos(1.2*t);