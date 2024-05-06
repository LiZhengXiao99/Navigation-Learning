function x=ckf_ProssEq(x)

n = size(x,2); %返回矩阵X的列数

for i=1:n
    
    x(:,i)=ProssEq(x(:,i));
    
end