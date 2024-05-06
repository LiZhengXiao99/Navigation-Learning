function [sigma]=embedded_sigma_point(delta)

v=[sqrt(2)*delta;-sqrt(2)*delta];

sigma=[];

for j1=1:2
    for j2=1:2
        e=[v(j1);v(j2)];
        sigma=[sigma e];
    end    
end


