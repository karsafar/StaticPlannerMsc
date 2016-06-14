
function simpson_val = integral_par_same_scale(par_int,flag, theta_ini,co_eff)

sections = 100;
sections = 8;
%flag = 1 choose the g function
if(flag)
    %simpson = integral(@g_function,0 , par_int(end),'AbsTol',1e-5,'RelTol',1e-4);
    simpson_val = simpsons(@g_function,0,par_int(end),sections);
else
    %simpson = integral(@f_function,0 ,par_int(end),'AbsTol',1e-5,'RelTol',1e-4);
    simpson_val = simpsons(@f_function,0,par_int(end),sections);
end


    function g_n = g_function(s)
        a=par_int(1);
        b = -(11*par_int(1) -18*par_int(2) +9*par_int(3) -2*par_int(4))/(2*par_int(end));
        c = 9*(2*par_int(1) - 5*par_int(2) + 4*par_int(3) - par_int(4))/(2*par_int(end)^2);
        d = -9*(par_int(1) -3*par_int(2) + 3*par_int(3) -par_int(4))/(2*par_int(end)^3);
        
        theta = a.*s + (b.*s.^2)/2 + (c.*s.^3)/(3) + (d.*s.^4)/(4) + theta_ini;
        %theta = par_int(1)*s + (par_int(2)*s^2)/2 + (par_int(3)*s^3)/3 + (par_int(4)*s^4)/4;
        %s^n*sin(theta)
        dthetadp = co_eff*[s.^2;s.^3;s.^4];
        g_n = dthetadp.*sin(theta);
        
    end
    function f_n = f_function(s)
        a=par_int(1);
        b = -(11*par_int(1) -18*par_int(2) +9*par_int(3) -2*par_int(4))/(2*par_int(end));
        c = 9*(2*par_int(1) - 5*par_int(2) + 4*par_int(3) - par_int(4))/(2*par_int(end)^2);
        d = -9*(par_int(1) -3*par_int(2) + 3*par_int(3) -par_int(4))/(2*par_int(end)^3);
        
        theta = a.*s + (b.*s.^2)/2 + (c.*s.^3)/(3) + (d.*s.^4)/(4) + theta_ini;
        %theta = par_int(1)*s + (par_int(2)*s^2)/2 + (par_int(3)*s^3)/3 + (par_int(4)*s^4)/4;
        %s^n*sin(theta)
        dthetadp = co_eff*[s.^2;s.^3;s.^4];
        %theta = par_int(1)*s + (par_int(2)*s^2)/2 + (par_int(3)*s^3)/3 + (par_int(4)*s^4)/4;
        %s^n*cos(theta)
        f_n = dthetadp.*cos(theta);    end
end
