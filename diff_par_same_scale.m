function jacobian_par = diff_par_same_scale(par_int, s, endpoint, thetaf, theta_ini)
%differentiating also wrt to a
% dxdq = [ -integral_par(1,par_int,1) -integral_par(2,par_int,1)/2 -integral_par(3,par_int,1)/3 -integral_par(4,par_int,1)/4 cos(cubic_theta(par_int,s))];
% dydq = [-integral_par(1,par_int,0) -integral_par(2,par_int,0)/2 -integral_par(3,par_int,0)/3 -integral_par(4,par_int,0)/4 sin(cubic_theta(par_int,s))];
% dthetadq =[s (s^2)/2 (s^3)/3 (s^4)/4 curvature_comp(par_int,s) ] ;
% dcurvaturedq = [1 s (s^2) (s^3) curvature_prime_comp(par_int)] ;



co_eff_p1 =[18/(4*par_int(end)) -45/(6*par_int(end)^2) 27/(8*par_int(end)^3)];
co_eff_p2 =[-9/(4*par_int(end)) 36/(6*par_int(end)^2) -27/(8*par_int(end)^3)];
dxdq = [ -integral_par_same_scale(par_int,1,theta_ini,co_eff_p1) -integral_par_same_scale(par_int,1, theta_ini,co_eff_p2) cos(thetaf + theta_ini)];
dydq = [integral_par_same_scale(par_int,0,theta_ini,co_eff_p1) integral_par_same_scale(par_int,0,theta_ini,co_eff_p2)  sin(thetaf + theta_ini)];
dthetadq =[((18/4)*s^2/par_int(end))-(45/6)*s^3/par_int(end)^2 + (27/8)*s^4/(par_int(end))^3 ((-9/4)*s^2/par_int(end))+(36/6)*s^3/par_int(end)^2 + (-27/8)*s^4/(par_int(end))^3 endpoint(end)] ;
%dcurvaturedq = [(18/2)-(45/2)+(27/2) (-9/2)+(36/2)-27/2 curvature_prime_comp_same_scale(par_int, par_int(end))] ;
%jacobian_par = [dxdq ;dydq ;dthetadq ;dcurvaturedq];
jacobian_par = [dxdq ;dydq ;dthetadq ];
end
