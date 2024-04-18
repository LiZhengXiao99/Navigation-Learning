function [varargout] = Substitute(parameters,current_point,varargin)

 for j=1:size(varargin,2)
     varargout{j}  = double(subs(varargin{j},parameters,current_point)); % substitute the parameter values
 end
end

