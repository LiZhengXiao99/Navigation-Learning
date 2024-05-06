function Eri=Calculate_Eri(sumri,alfa,beta)

rgm=0.01:0.01:(1-sumri);

% gamma=gampdf(rgm,alfa,1/beta)+1e-98;

gamma=gampdf(rgm,alfa,1/beta);

ri=0.01*rgm.*(gamma/(sum(gamma)*0.01)); 

Eri=sum(ri);

end

