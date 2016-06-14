function y_d_prime = central_diff_2(V,h)
y_d_prime = ones(1,length(V));
y_d_prime(2:end-1) = (V(1:end-2)-2*V(2:end-1) + V(3:end))/h^2;
%forward difference for first point
y_d_prime(1) = (V(3)-2*V(2) + V(1))/h^2;
%backward difference
y_d_prime(end) = (V(end)-2*V(end-1) + V(end-2))/h^2;

end