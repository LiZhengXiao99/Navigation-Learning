function [E_y,E_y2,E_k2_y,E_log_y]=E_GGSM(Lamda_A,m,delta_1,delta_2,delta_3)

M=size(Lamda_A,2);

w=zeros(1,M);

for j=1:M
    
    y=Lamda_A(j);
    
    log_w_j=-0.5*m*delta_1*log(y)-0.5*delta_2/y-0.5*delta_3*y; % k2_lamada=1/lamada
    
    w(j)=exp(log_w_j);

end

%%%%Weight Normalization
w=w/sum(w);

%%%%Calculate Expectations
E_y=Lamda_A*w';

E_y2=(Lamda_A.*Lamda_A)*w';

E_k2_y=(1./Lamda_A)*w';

E_log_y=log(Lamda_A)*w';

end

