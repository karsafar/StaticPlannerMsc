function [param, counter, delta_state_index] = compute_path_quartic(startpoint_norm,endpoint_norm)


% 
% a = param_old(1);
% b = param_old(2);
% c = -(85*param_old(1) - 108*param_old(3) + 27*param_old(4) - 4*param_old(5) + 22*param_old(2)*sf)/(4*param_old(end)^2);
% 
% d = (9*(10*param_old(1) - 15*param_old(3) + 6*param_old(4) - param_old(5) + 2*param_old(2)*sf))/(2*param_old(end)^3);
% 
% 
% e = -(9*(11*param_old(1) - 18*param_old(3) + 9*param_old(4) - 2*param_old(5) + 2*param_old(2)*sf))/(4*param_old(end)^4);


param_old = zeros(6,1);
draw = 0;
step = 16;
param_old(3) =0;
param_old(4) =0;
param_old(6) = endpoint_norm(1);
curvature_ini = startpoint_norm(4);

kprime_ini = startpoint_norm(5);
x = endpoint_norm(1);
y = endpoint_norm(2);
thetaf = endpoint_norm(3);
curvature_f = endpoint_norm(4);
theta_ini =0;
scale = 1;
tolerance = 0.01;
%weights = [ 1; 1; 100; 100];
weights = [ 1; 1; 100];

for inc =1:16
    param_old(1)= inc/step*curvature_ini;
    param_old(2) = inc/step*kprime_ini;
    param_old(5) = inc/step*curvature_f;
    startpoint = [0;0;0;inc/step*curvature_ini;inc/step*kprime_ini];
    endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
    
    a = param_old(1);
    b = param_old(2);
    c = -(85*param_old(1) - 108*param_old(3) + 27*param_old(4) - 4*param_old(5) + 22*param_old(2)*param_old(end))/(4*param_old(end)^2);
    
    d = (9*(10*param_old(1) - 15*param_old(3) + 6*param_old(4) - param_old(5) + 2*param_old(2)*param_old(end)))/(2*param_old(end)^3);
    
    
    e = -(9*(11*param_old(1) - 18*param_old(3) + 9*param_old(4) - 2*param_old(5) + 2*param_old(2)*param_old(end)))/(4*param_old(end)^4);
    param_abcd = [ a ; b;c;d;e];
    %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
    %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
    delta_state = [integral_par_quartic(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par_quartic(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; quartic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
    delta_state_index = sqrt(sum((weights.*delta_state).^2));
    counter = 0;
    
    damping = 0;
    param = param_old;
    while(delta_state_index> tolerance)&&(counter<30)
        counter = counter + 1;
        
        %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
        delta_param = -( diff_par_quartic(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
        
        param = param_old + [0;0;delta_param(1:2);0;delta_param(end)];
        
        a = param(1);
        b = param(2);
        c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);
        
        d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);
        
        
        e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);
        param_abcd = [ a ; b;c;d;e];
        %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
        %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
        delta_state = [integral_par_quartic(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par_quartic(0,[param_abcd; param(end)],1,scale, theta_ini) ; quartic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
        
        
        delta_state_index = sqrt(sum((weights.*delta_state).^2));
        %     if(delta_state_index<= delta_state_index_prev)
        %         damping = damping/10;
        %     else
        %         damping = damping*10;
        %     end
        %      delta_state_index_prev = delta_state_index;
        param_old = param;
    end
    if(draw)
    [x1 y1]= drawtrajec_abcde(param,startpoint,endpoint,20);
    figure(3) 
    plot(x1,y1)
    hold on
    end
    
    
    
    
    
    
    
end