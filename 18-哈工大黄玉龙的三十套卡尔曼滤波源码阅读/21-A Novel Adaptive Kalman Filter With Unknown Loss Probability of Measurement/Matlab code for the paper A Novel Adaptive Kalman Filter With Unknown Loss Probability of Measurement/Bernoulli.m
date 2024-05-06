function [Theta] = Bernoulli(p)
% 0=<p<=1;

x=rand;

if x<p
    
    Theta=1;
    
else
    
    Theta=0;
    
end
