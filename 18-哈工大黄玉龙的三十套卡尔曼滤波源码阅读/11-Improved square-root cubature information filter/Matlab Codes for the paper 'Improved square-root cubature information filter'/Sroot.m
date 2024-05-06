function S=Sroot(A)
[Q,R]=qr(A',0);

S=R';