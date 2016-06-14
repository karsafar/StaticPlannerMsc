function theta = quartic_theta(par_int,s,scale,theta_ini)
theta = par_int(1).*s/scale + (par_int(2).*s.^2)/(scale^2*2) + (par_int(3).*s.^3)/(scale^3*3) + (par_int(4).*s.^4)/(scale^4*4)+ (par_int(5).*s.^5)/(scale^5*5) + theta_ini;
end