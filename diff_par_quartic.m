function jacobian_par = diff_par_quartic(par_int, s, endpoint, thetaf, theta_ini)
%differentiating also wrt to a
% dxdq = [ -integral_par(1,par_int,1) -integral_par(2,par_int,1)/2 -integral_par(3,par_int,1)/3 -integral_par(4,par_int,1)/4 cos(cubic_theta(par_int,s))];
% dydq = [-integral_par(1,par_int,0) -integral_par(2,par_int,0)/2 -integral_par(3,par_int,0)/3 -integral_par(4,par_int,0)/4 sin(cubic_theta(par_int,s))];
% dthetadq =[s (s^2)/2 (s^3)/3 (s^4)/4 curvature_comp(par_int,s) ] ;
% dcurvaturedq = [1 s (s^2) (s^3) curvature_prime_comp(par_int)] ;



co_eff_p3 =[108/(12*par_int(end)^2) -135/(8*par_int(end)^3) 162/(20*par_int(end)^4)];
co_eff_p4 =[-27/(12*par_int(end)^2) 54/(8*par_int(end)^3) -81/(20*par_int(end)^4)];
dxdq = [ -integral_par_same_scale_quart(par_int,1,theta_ini,co_eff_p3) -integral_par_same_scale_quart(par_int,1, theta_ini,co_eff_p4) cos(thetaf + theta_ini)];
dydq = [integral_par_same_scale_quart(par_int,0,theta_ini,co_eff_p3) integral_par_same_scale_quart(par_int,0,theta_ini,co_eff_p4)  sin(thetaf + theta_ini)];
dthetadq =[co_eff_p3*[s.^3;s.^4;s.^5] co_eff_p4*[s.^3;s.^4;s.^5] endpoint(end)] ;
%dcurvaturedq = [(18/2)-(45/2)+(27/2) (-9/2)+(36/2)-27/2 curvature_prime_comp_same_scale(par_int, par_int(end))] ;
%jacobian_par = [dxdq ;dydq ;dthetadq ;dcurvaturedq];
jacobian_par = [dxdq ;dydq ;dthetadq ];
end
