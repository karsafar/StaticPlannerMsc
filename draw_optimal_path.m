function draw_optimal_path(optimal_par,start_car,lattice, draw_car_flag, draw_path)

start_pose = start_car.start;
param_matrix_start = start_car.traj;
dimension = start_car.dimension;
position_x = lattice.x;
position_y = lattice.y;
orientation = lattice.theta;
curvature = lattice.curve;
param_matrix = lattice.param ;
v_vec = lattice.vel ;

%draw optimal
time_acc = 0;
time_start = 0;



velocity_cont = [];
velocity_cont_t = [];
path_x=[];
path_y =[];
s_cont =[];
%draw trajectory from startpose
%compute endpoint of start trajecotry which is on the state lattice
%endpoint = [position_x(optimal_par(2,2) , 1); position_y(optimal_par(2,2), 1); orientation(optimal_par(2,2),1);curvature(optimal_par(2,2), 1)];
% [x y]= drawtrajec_abcde(param_matrix_start(optimal_par(2,2), 1 :6)' ,start_pose(1:4),endpoint,20);
% figure(1)
% hold on

%plot(x,y,'-ko')




%draw every 0.1s
step = 0.1;
remainder = step;
%generate velocity profile
[s,T, acc, jerk, v,rem] = velgen_draw(start_pose(6),v_vec(optimal_par(3,2)),start_pose(7),0,param_matrix_start(optimal_par(2,2),6)',step,remainder);



param = param_matrix_start(optimal_par(2,2),1:6);
%convert from the p1 to p5 parameters to abcde
a = param(1);
b = param(2);
c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);
d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);

e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);
param_abcde = [ a ; b;c;d;e];
s_cont = param(end);


%compute the orientation of the car
theta = quartic_theta([a;b;c;d;e],s,1,start_pose(3));

figure(1)
hold on
if(s(1)==0)
    path_x = [start_pose(1)];
    path_y = [start_pose(2)];
else
    path_x =(integral_par_quartic(0,[param_abcde;s(1)],0,1,start_pose(3)) + start_pose(1));
    path_y= (integral_par_quartic(0,[param_abcde;s(1)],1,1,start_pose(3)) + start_pose(2));
end
%draw car per section
    draw_car(dimension,[path_x(1) path_y(1) theta(1)/pi*180])

if(draw_car_flag)
    %draw car per section
    draw_car(dimension,[path_x path_y theta(1)/pi*180])
end
for j = 2: length(s)
    x1 =(integral_par_quartic(0,[param_abcde;s(j)],0,1,start_pose(3)) + start_pose(1));
    y1= (integral_par_quartic(0,[param_abcde;s(j)],1,1,start_pose(3)) + start_pose(2));
    path_x = [path_x x1];
    path_y = [ path_y y1];
    if(draw_car_flag)
        % draw car per section
        draw_car(dimension,[ x1 y1 theta(j)*180/pi])
    end
end
if(draw_path)
    %plot(path_x,path_y,'-k*','LineWidth',1.5);
end
%create time vector
t= time_start:step:T+time_acc;

velocity_cont= [velocity_cont v];
velocity_cont_t = [velocity_cont_t t];

time_start = t(end)+step;
time_acc = time_acc + T;
% figure(3)
% plot(t,test,'-*g')
% hold on
% figure(4)
% plot(t,testy,'-*bl')
% hold on
t =[];


%path_x = [];
%path_y = [];






for i = 2: length(optimal_par)-1
    if(~isinf(optimal_par(1,i+1)))
        startpoint = [position_x(optimal_par(2,i),i-1); position_y(optimal_par(2,i),i-1); orientation(optimal_par(2,i),i-1);curvature(optimal_par(2,i),i-1)];
        %endpoint = [position_x(optimal_par(2,i+1) , i ); position_y(optimal_par(2,i +1), i ); orientation(optimal_par(2,i+1),i );curvature(optimal_par(2,i+1), i)];
        %     [x y theta1 curvature1 spacing]= drawtrajec_abcd(param_matrix(optimal_par(2,i+1),(i-2)*7 +1 :(i-2)*7 +5, optimal_par(2,i))' ,startpoint,endpoint,20);
        %
        %
        %     figure(1)
        %     hold on
        %
        %     plot(x,y,'-ko')
        %draw every 0.1s
        step = 0.1;
        [s,T, a, jerk, v, rem] = velgen_draw(v_vec(optimal_par(3,i)),v_vec(optimal_par(3,i+1)),0,0,param_matrix(optimal_par(2,i+1),(i-2)*7 +5, optimal_par(2,i))',0.1,rem);
        theta = cubic_theta_same_scale(param_matrix(optimal_par(2,i+1),(i-2)*7 +1 :(i-2)*7 +5, optimal_par(2,i))',s,orientation(optimal_par(2,i),i-1));
        % draw_car(dimension,[startpoint(1) startpoint(2) theta(1)/pi*180])
        param = param_matrix(optimal_par(2,i+1),(i-2)*7 +1 :(i-2)*7 +5, optimal_par(2,i));
        
        a=param(1);
        b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
        c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
        d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
        param_abcd = [ a ; b;c;d];
        s_cont =[ s_cont param(end)];
        if(s(1)==0)
            path_x = [path_x startpoint(1)];
            path_y = [path_y startpoint(2)];
        else
            path_x =[path_x (integral_par(0,[param_abcd;s(1)],0,1,startpoint(3)) + startpoint(1))];
            path_y= [path_y (integral_par(0,[param_abcd;s(1)],1,1,startpoint(3)) + startpoint(2))];
        end
                    draw_car(dimension,[startpoint(1) startpoint(2) theta(1)/pi*180])

        if(draw_car_flag)
            %draw a car per section
            draw_car(dimension,[test testy theta(1)/pi*180])
        end
        for j = 2: length(s)
            
            x1 =(integral_par(0,[param_abcd;s(j)],0,1,startpoint(3)) + startpoint(1));
            y1= (integral_par(0,[param_abcd;s(j)],1,1,startpoint(3)) + startpoint(2));
            path_x = [path_x x1];
            path_y = [path_y y1];
            if(draw_car_flag)
                % draw car per section
                draw_car(dimension,[ x1 y1 theta(j)*180/pi])
            end
        end
        if(draw_path)
          %  plot(path_x,path_y,'-k*','LineWidth',1.5);
        end
        t= time_start:step:T+time_acc;
        
        velocity_cont= [velocity_cont v];
        velocity_cont_t = [velocity_cont_t t];
        
        time_start = t(end)+step;
        time_acc = time_acc + T;
        
        
        t =[];
     %   path_x = [];
      %  path_y = [];
    end
end
plot(path_x,path_y,'-k','LineWidth',1.5);
figure(2)
plot(velocity_cont_t, velocity_cont,'-b','LineWidth',1.5);
hold on
xlabel('time(s)','FontSize',14)
ylabel('velocity(m/s)','FontSize',14)
%title('The graph of velocity(m/s) against time(s)','FontSize',16)

end