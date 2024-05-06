function Pu=fun_pu(c,r)

%%%%%%%%%%%%%%%%%
n=size(c,1);

pu=zeros(n,1);

%%%%%%%%%%%%%%%%%%
for i=1:n
    
    ci=c(i);
    
    %%%%%%%%%%%%%Huber
    if abs(ci)<r
        pu(i)=1;
    end
    
    if abs(ci)>=r
        if ci>0
            pu(i)=r/ci;
        else
            pu(i)=-r/ci;
        end
    end

end

Pu=diag(pu);