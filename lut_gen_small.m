function lut = lut_gen_small(sign_lat,sign_theta)
%curvature range +- 0.2
%theta range +- 90deg
%x yrange 1-50m

step = 200;
%initial guess
draw = 0;

theta_ini = 0;
i= 0;
scale = 1;
tolerance = 0.01;
%weights = [ 1; 1; 100; 100];
weights = [ 1; 1; 100];
param_old = zeros(5,1);
param_ini =[];
for x = 8:8
    i = i+1;
    j = 0;
    % for y = 0.5:0.25:7
    for y = sign_lat*0.75:sign_lat*0.75:sign_lat*8
        j = j +1;
        k =0;
        
        
        
        
        
        
        
        
        
        
        
        
        for thetaf = 0:sign_theta*0.04:sign_theta*pi/3
            k = k+1;
            
            l = 21;
            
            
            
            
            %initial trajectory
            
            
            m =21;
            curvature_ini = 0;
            curvature_f= 0;
            if (isempty(param_ini))||(k ==1)
                param_old(2) =0;
                param_old(3) =0;
                param_old(5) = x;
                for inc =1:200
                    param_old(1)= inc/step*curvature_ini;
                    param_old(4) = inc/step*curvature_f;
                    startpoint = [0;0;0;inc/step*curvature_ini];
                    endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                    
                    a=param_old(1);
                    b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                    c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                    d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                    param_abcd = [ a ; b;c;d];
                    %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    counter = 0;
                    
                    damping = 0;
                    param = param_old;
                    while(delta_state_index> tolerance)&&(counter<30)
                        counter = counter + 1;
                        
                        %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                        delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                        
                        param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                        
                        a=param(1);
                        b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                        c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                        d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                        
                        
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        %     if(delta_state_index<= delta_state_index_prev)
                        %         damping = damping/10;
                        %     else
                        %         damping = damping*10;
                        %     end
                        %      delta_state_index_prev = delta_state_index;
                        param_old = param;
                    end
                    
                    [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    if(draw)
                        hold on
                        plot(x1,y1)
                    end
                end
                lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                param_ini = param;
                param_ini_temp = param_ini;
                
            else
                
                param_old = param_ini;
                param_old(1)= curvature_ini;
                param_old(4) = curvature_f;
                startpoint = [0;0;0;curvature_ini];
                endpoint = [x;y;thetaf;curvature_f];
                
                
                
                a=param_old(1);
                b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                param_abcd = [ a ; b;c;d];
                %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                delta_state_index = sqrt(sum((weights.*delta_state).^2));
                
                if(delta_state_index < 20 )
                    counter = 0;
                    
                    damping = 0;
                    param = param_old;
                    while(delta_state_index> tolerance)&&(counter<30)
                        counter = counter + 1;
                        
                        %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                        delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                        
                        param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                        
                        a=param(1);
                        b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                        c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                        d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf)];
                        
                        
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        %     if(delta_state_index<= delta_state_index_prev)
                        %         damping = damping/10;
                        %     else
                        %         damping = damping*10;
                        %     end
                        %      delta_state_index_prev = delta_state_index;
                        param_old = param;
                    end
                    
                    [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    if(draw)
                        hold on
                        plot(x1,y1)
                    end
                    
                    lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                    param_ini = param;
                    param_ini_temp = param_ini;
                else
                    
                    param_old(2) =0;
                    param_old(3) =0;
                    param_old(5) = x;
                    for inc =1:200
                        param_old(1)= inc/step*curvature_ini;
                        param_old(4) = inc/step*curvature_f;
                        startpoint = [0;0;0;inc/step*curvature_ini];
                        endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                        
                        a=param_old(1);
                        b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                        c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                        d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        counter = 0;
                        
                        damping = 0;
                        param = param_old;
                        while(delta_state_index> tolerance)&&(counter<30)
                            counter = counter + 1;
                            
                            %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                            delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                            
                            param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                            
                            a=param(1);
                            b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                            c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                            d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                            
                            
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                            %     if(delta_state_index<= delta_state_index_prev)
                            %         damping = damping/10;
                            %     else
                            %         damping = damping*10;
                            %     end
                            %      delta_state_index_prev = delta_state_index;
                            param_old = param;
                        end
                        
                        [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                        if(draw)
                            hold on
                            plot(x1,y1)
                        end
                    end
                    lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                    param_ini = param;
                    param_ini_temp = param_ini;
                end
                
            end
            
            l =22;
            for curvature_ini = 0:-0.01:-0.2
                l = l-1;
                m = 22;
                param_old = param_ini_temp;
                for curvature_f = 0:-0.01:-0.2
                    m =m -1;
                    
                    %                     param_old(2) =0;
                    %                     param_old(3) =0;
                    %                     param_old(5) = x;
                    %                     for inc =1:16
                    %                         param_old(1)= inc/step*curvature_ini;
                    %                         param_old(4) = inc/step*curvature_f;
                    %                         startpoint = [0;0;0;inc/step*curvature_ini];
                    %                         endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                    %
                    %                         a=param_old(1);
                    %                         b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                    %                         c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                    %                         d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                    %                         param_abcd = [ a ; b;c;d];
                    %                         %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                         %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                         delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %                         delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                         counter = 0;
                    %
                    %                         damping = 0;
                    %
                    %                         while(delta_state_index> tolerance)&&(counter<10)
                    %                             counter = counter + 1;
                    %
                    %                             %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %                             delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %
                    %                             param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                    %
                    %                             a=param(1);
                    %                             b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    %                             c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    %                             d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    %                             param_abcd = [ a ; b;c;d];
                    %                             %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                             %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                             delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %
                    %
                    %                             delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                             %     if(delta_state_index<= delta_state_index_prev)
                    %                             %         damping = damping/10;
                    %                             %     else
                    %                             %         damping = damping*10;
                    %                             %     end
                    %                             %      delta_state_index_prev = delta_state_index;
                    %                             param_old = param;
                    %                         end
                    %
                    %                         [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    %                     hold on
                    %                     plot(x1,y1)
                    %                     end
                    
                    if~((l==21)&&(m==21))
                        
                        param_old(1)= curvature_ini;
                        param_old(4) = curvature_f;
                        startpoint = [0;0;0;curvature_ini];
                        endpoint = [x;y;thetaf;curvature_f];
                        
                        
                        
                        a=param_old(1);
                        b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                        c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                        d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        if(delta_state_index>20)
                            param_old = param_ini_temp;
                            param_old(1)= curvature_ini;
                            param_old(4) = curvature_f;
                            startpoint = [0;0;0;curvature_ini];
                            endpoint = [x;y;thetaf;curvature_f];
                            
                            
                            
                            a=param_old(1);
                            b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                            c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                            d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        end
                        
                        counter = 0;
                        
                        damping = 0;
                        param = param_old;
                        while(delta_state_index> tolerance)&&(counter<30)
                            counter = counter + 1;
                            
                            %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                            delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                            
                            param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                            
                            a=param(1);
                            b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                            c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                            d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                            
                            
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                            %     if(delta_state_index<= delta_state_index_prev)
                            %         damping = damping/10;
                            %     else
                            %         damping = damping*10;
                            %     end
                            %      delta_state_index_prev = delta_state_index;
                            param_old = param;
                        end
                        
                        [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                        if(draw)
                            hold on
                            plot(x1,y1)
                        end
                        
                        lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                        if(m ==21)&&(delta_state_index<0.4)
                            param_ini_temp = param;
                        end
                    end
                    
                end
                
            end
            
            
            
            l =22;
            param_ini_temp = param_ini;
            for curvature_ini = 0:-0.01:-0.2
                l = l-1;
                m = 20;
                param_old = param_ini_temp;
                for curvature_f = 0:0.01:0.2
                    m =m + 1;
                    
                    %                     param_old(2) =0;
                    %                     param_old(3) =0;
                    %                     param_old(5) = x;
                    %                     for inc =1:16
                    %                         param_old(1)= inc/step*curvature_ini;
                    %                         param_old(4) = inc/step*curvature_f;
                    %                         startpoint = [0;0;0;inc/step*curvature_ini];
                    %                         endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                    %
                    %                         a=param_old(1);
                    %                         b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                    %                         c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                    %                         d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                    %                         param_abcd = [ a ; b;c;d];
                    %                         %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                         %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                         delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %                         delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                         counter = 0;
                    %
                    %                         damping = 0;
                    %
                    %                         while(delta_state_index> tolerance)&&(counter<10)
                    %                             counter = counter + 1;
                    %
                    %                             %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %                             delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %
                    %                             param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                    %
                    %                             a=param(1);
                    %                             b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    %                             c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    %                             d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    %                             param_abcd = [ a ; b;c;d];
                    %                             %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                             %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                             delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %
                    %
                    %                             delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                             %     if(delta_state_index<= delta_state_index_prev)
                    %                             %         damping = damping/10;
                    %                             %     else
                    %                             %         damping = damping*10;
                    %                             %     end
                    %                             %      delta_state_index_prev = delta_state_index;
                    %                             param_old = param;
                    %                         end
                    %
                    %                         [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    %                     hold on
                    %                     plot(x1,y1)
                    %                     end
                    
                    if~((l==21)&&(m==21))
                        
                        param_old(1)= curvature_ini;
                        param_old(4) = curvature_f;
                        startpoint = [0;0;0;curvature_ini];
                        endpoint = [x;y;thetaf;curvature_f];
                        
                        
                        
                        a=param_old(1);
                        b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                        c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                        d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        if(delta_state_index>20)
                            param_old = param_ini_temp;
                            param_old(1)= curvature_ini;
                            param_old(4) = curvature_f;
                            startpoint = [0;0;0;curvature_ini];
                            endpoint = [x;y;thetaf;curvature_f];
                            
                            
                            
                            a=param_old(1);
                            b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                            c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                            d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        end
                        
                        counter = 0;
                        
                        damping = 0;
                        param = param_old;
                        while(delta_state_index> tolerance)&&(counter<30)
                            counter = counter + 1;
                            
                            %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                            delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                            
                            param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                            
                            a=param(1);
                            b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                            c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                            d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                            
                            
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                            %     if(delta_state_index<= delta_state_index_prev)
                            %         damping = damping/10;
                            %     else
                            %         damping = damping*10;
                            %     end
                            %      delta_state_index_prev = delta_state_index;
                            param_old = param;
                        end
                        
                        [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                        if(draw)
                            hold on
                            plot(x1,y1)
                        end
                        
                        lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                        if(m ==21)&&(delta_state_index<0.4)
                            param_ini_temp = param;
                        end
                    end
                    
                end
                
            end
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            l = 20;
            param_ini_temp = param_ini;
            for curvature_ini = 0:0.01:0.2
                l = l+1;
                m = 20;
                param_old = param_ini_temp;
                for curvature_f = 0:0.01:0.2
                    m =m +1;
                    
                    %                     param_old(2) =0;
                    %                     param_old(3) =0;
                    %                     param_old(5) = x;
                    %                     for inc =1:16
                    %                         param_old(1)= inc/step*curvature_ini;
                    %                         param_old(4) = inc/step*curvature_f;
                    %                         startpoint = [0;0;0;inc/step*curvature_ini];
                    %                         endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                    %
                    %                         a=param_old(1);
                    %                         b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                    %                         c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                    %                         d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                    %                         param_abcd = [ a ; b;c;d];
                    %                         %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                         %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                         delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %                         delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                         counter = 0;
                    %
                    %                         damping = 0;
                    %
                    %                         while(delta_state_index> tolerance)&&(counter<10)
                    %                             counter = counter + 1;
                    %
                    %                             %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %                             delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %
                    %                             param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                    %
                    %                             a=param(1);
                    %                             b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    %                             c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    %                             d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    %                             param_abcd = [ a ; b;c;d];
                    %                             %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                             %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                             delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %
                    %
                    %                             delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                             %     if(delta_state_index<= delta_state_index_prev)
                    %                             %         damping = damping/10;
                    %                             %     else
                    %                             %         damping = damping*10;
                    %                             %     end
                    %                             %      delta_state_index_prev = delta_state_index;
                    %                             param_old = param;
                    %                         end
                    %
                    %                         [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    %                     hold on
                    %                     plot(x1,y1)
                    %                     end
                    
                    if~((l==21)&&(m==21))
                        
                        param_old(1)= curvature_ini;
                        param_old(4) = curvature_f;
                        startpoint = [0;0;0;curvature_ini];
                        endpoint = [x;y;thetaf;curvature_f];
                        
                        
                        
                        a=param_old(1);
                        b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                        c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                        d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        if(delta_state_index>20)
                            param_old = param_ini_temp;
                            param_old(1)= curvature_ini;
                            param_old(4) = curvature_f;
                            startpoint = [0;0;0;curvature_ini];
                            endpoint = [x;y;thetaf;curvature_f];
                            
                            
                            
                            a=param_old(1);
                            b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                            c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                            d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        end
                        
                        counter = 0;
                        
                        damping = 0;
                        param = param_old;
                        while(delta_state_index> tolerance)&&(counter<30)
                            counter = counter + 1;
                            
                            %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                            delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                            
                            param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                            
                            a=param(1);
                            b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                            c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                            d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                            
                            
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                            %     if(delta_state_index<= delta_state_index_prev)
                            %         damping = damping/10;
                            %     else
                            %         damping = damping*10;
                            %     end
                            %      delta_state_index_prev = delta_state_index;
                            param_old = param;
                        end
                        
                        [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                        if(draw)
                            hold on
                            plot(x1,y1)
                        end
                        
                        lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                        if(m ==21)&&(delta_state_index<0.4)
                            param_ini_temp = param;
                        end
                    end
                    
                end
                
            end
            
            l = 20;
            param_ini_temp = param_ini;
            for curvature_ini = 0:0.01:0.2
                l = l+1;
                m = 22;
                param_old = param_ini_temp;
                for curvature_f = 0:-0.01:-0.2
                    m =m -1;
                    
                    %                     param_old(2) =0;
                    %                     param_old(3) =0;
                    %                     param_old(5) = x;
                    %                     for inc =1:16
                    %                         param_old(1)= inc/step*curvature_ini;
                    %                         param_old(4) = inc/step*curvature_f;
                    %                         startpoint = [0;0;0;inc/step*curvature_ini];
                    %                         endpoint = [x;inc/step*y;inc/step*thetaf;inc/step*curvature_f];
                    %
                    %                         a=param_old(1);
                    %                         b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                    %                         c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                    %                         d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                    %                         param_abcd = [ a ; b;c;d];
                    %                         %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                         %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                         delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %                         delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                         counter = 0;
                    %
                    %                         damping = 0;
                    %
                    %                         while(delta_state_index> tolerance)&&(counter<10)
                    %                             counter = counter + 1;
                    %
                    %                             %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %                             delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                    %
                    %                             param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                    %
                    %                             a=param(1);
                    %                             b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    %                             c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    %                             d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    %                             param_abcd = [ a ; b;c;d];
                    %                             %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                    %                             %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                    %                             delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                    %
                    %
                    %                             delta_state_index = sqrt(sum((weights.*delta_state).^2));
                    %                             %     if(delta_state_index<= delta_state_index_prev)
                    %                             %         damping = damping/10;
                    %                             %     else
                    %                             %         damping = damping*10;
                    %                             %     end
                    %                             %      delta_state_index_prev = delta_state_index;
                    %                             param_old = param;
                    %                         end
                    %
                    %                         [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                    %                     hold on
                    %                     plot(x1,y1)
                    %                     end
                    
                    if~((l==21)&&(m==21))
                        
                        param_old(1)= curvature_ini;
                        param_old(4) = curvature_f;
                        startpoint = [0;0;0;curvature_ini];
                        endpoint = [x;y;thetaf;curvature_f];
                        
                        
                        
                        a=param_old(1);
                        b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                        c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                        d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                        param_abcd = [ a ; b;c;d];
                        %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                        %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                        delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                        delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        if(delta_state_index>20)
                            param_old = param_ini_temp;
                            param_old(1)= curvature_ini;
                            param_old(4) = curvature_f;
                            startpoint = [0;0;0;curvature_ini];
                            endpoint = [x;y;thetaf;curvature_f];
                            
                            
                            
                            a=param_old(1);
                            b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
                            c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
                            d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param_old,0,scale); integral_par(0,param_old,1,scale) ; cubic_theta(param_old,param_old(end),scale); curvature_comp(param_old,param_old(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini); curvature_comp(param_abcd,param_old(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+ thetaf)];
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                        end
                        
                        counter = 0;
                        
                        damping = 0;
                        param = param_old;
                        while(delta_state_index> tolerance)&&(counter<30)
                            counter = counter + 1;
                            
                            %delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
                            delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,inc/step*thetaf, theta_ini)+damping*eye(3))\delta_state;
                            
                            param = param_old + [0;delta_param(1:2);0;delta_param(end)];
                            
                            a=param(1);
                            b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                            c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                            d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                            param_abcd = [ a ; b;c;d];
                            %delta_state = [integral_par(0,param,0,scale); integral_par(0,param,1,scale) ; cubic_theta(param,param(end),scale); curvature_comp(param,param(end),scale)] + [startpoint(1:end-1); 0]- endpoint;
                            %delta_state = [integral_par(0,[param_abcd ; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd ; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini); curvature_comp(param_abcd,param(end),scale)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf); -endpoint(end)];
                            delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+inc/step*thetaf)];
                            
                            
                            delta_state_index = sqrt(sum((weights.*delta_state).^2));
                            %     if(delta_state_index<= delta_state_index_prev)
                            %         damping = damping/10;
                            %     else
                            %         damping = damping*10;
                            %     end
                            %      delta_state_index_prev = delta_state_index;
                            param_old = param;
                        end
                        
                        [x1 y1 theta1 curvature1 spacing]= drawtrajec_abcd(param,startpoint,endpoint,20);
                        if(draw)
                            hold on
                            plot(x1,y1)
                        end
                        
                        lut((i-1)*((8-0.75)/0.25 + 1)*27*41*41+(j-1)*27*41*41+(k-1)*41*41+(l-1)*41+ m,:) = [x y thetaf curvature_ini curvature_f param' delta_state_index];
                        if(m ==21)&&(delta_state_index<0.4)
                            param_ini_temp = param;
                        end
                    end
                    
                end
                
            end
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        end
    end
end
end

