function dy=C_IF(y,delta)

n=size(y,2);

dy=zeros(1,n-1);

for i=1:(n-1)
    
    yi=-log(y(i));
    
    yi_1=-log(y(i+1));
    
    dy(i)=(yi_1-yi)/delta;

end

end

