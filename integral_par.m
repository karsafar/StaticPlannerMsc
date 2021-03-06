function simpson_val = integral_par(n,par_int,flag,scale, theta_ini)
%flag = 1 choose the g function 
sections = 100;
if(flag)
   % simpson = integral(@g_function,0 , par_int(end)/scale,'AbsTol',1e-5,'RelTol',1e-4);
    simpson_val = simpsons(@g_function,0,par_int(end)/scale,sections);
else
    %simpson = integral(@f_function,0 ,par_int(end)/scale,'AbsTol',1e-5,'RelTol',1e-4);
    simpson_val = simpsons(@f_function,0,par_int(end)/scale,sections);
end


    function g_n = g_function(s)
        %theta = par_int(1)*s + (par_int(2)*s^2)/2 + (par_int(3)*s^3)/3 + (par_int(4)*s^4)/4;
        %s^n*sin(theta)
        g_n = s.^n.*sin(par_int(1).*s/scale + (par_int(2).*s.^2)/(scale^2*2) + (par_int(3).*s.^3)/(scale^3*3) + (par_int(4).*s.^4)/(scale^4*4) + theta_ini);
    end
    function f_n = f_function(s)
        
        %theta = par_int(1)*s + (par_int(2)*s^2)/2 + (par_int(3)*s^3)/3 + (par_int(4)*s^4)/4;
       %s^n*cos(theta)
        f_n = s.^n.*cos(par_int(1).*s/scale + (par_int(2).*s.^2)/(scale^2*2) + (par_int(3).*s.^3)/(scale^3*3) + (par_int(4).*s.^4)/(scale^4*4) +theta_ini);
    end
end