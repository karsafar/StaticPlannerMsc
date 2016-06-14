function y_prime = central_diff_1(V,h)
%inspired from http://www.mathworks.com/matlabcentral/fileexchange/12-central-diff-m
y_prime = ones(1,length(V));
y_prime(2:end-1) = (V(3:end)-V(1:end-2))/(2*h);
%forward difference
y_prime(1) = (V(2)-V(1))/h;
%backward difference
y_prime(end) = (V(end) - V(end-1))/h;
end