function theta = cubic_theta_same_scale(par_int,s,theta_ini)
a=par_int(1);
b = -(11*par_int(1) -18*par_int(2) +9*par_int(3) -2*par_int(4))/(2*par_int(end));
c = 9*(2*par_int(1) - 5*par_int(2) + 4*par_int(3) - par_int(4))/(2*par_int(end)^2);
d = -9*(par_int(1) -3*par_int(2) + 3*par_int(3) -par_int(4))/(2*par_int(end)^3);

theta = a.*s + (b.*s.^2)/2 + (c.*s.^3)/(3) + (d.*s.^4)/(4) + theta_ini;
end