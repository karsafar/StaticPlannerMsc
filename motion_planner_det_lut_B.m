function optimal_par = motion_planner_det_lut_B(prior_info,dimension, start_pose,spacing_param, static_obstacle_matrix, initial_guess,n_path)


% define maximum curvature
max_curv = 1/3.8;
%load initial guess tables
lut_neg_lat = initial_guess.neg_lat;
lut_neg = initial_guess.neg_ang;
lut = initial_guess.pos;
lut_neg_double = initial_guess.neg_double;

sub_sample = spacing_param.sub_sample ;
station_new = spacing_param.station ;

draw = 1;

sub_centre_line = prior_info.prior_centre_line;

%%%%%%  change lateral distance to adjust the width of the lattice %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lateral_dist = 1 ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_path_vec = add_variance_con(sub_centre_line(:,2:end),lateral_dist,n_path); % instead of n_path should be 5
%n_path_vec = n_path;
n_path = max(n_path_vec);

lateral_dist_vec = -lateral_dist*(n_path-1)/2:lateral_dist:lateral_dist*(n_path-1)/2;
lateral_dist_mat = repmat(lateral_dist_vec',1,size(sub_centre_line,2)-1);
position_x = repmat(sub_centre_line(1,2:end),n_path,1)  +  lateral_dist_mat.*repmat(cos(sub_centre_line(3,2:end) + pi/2),n_path,1);
position_y = repmat(sub_centre_line(2,2:end),n_path,1)  +  lateral_dist_mat.*repmat(sin(sub_centre_line(3,2:end) + pi/2),n_path,1);

orientation = repmat([sub_centre_line(3,2:end)], n_path,1);
curvature = 1./(1./repmat(sub_centre_line(4,2:end), n_path, 1) -lateral_dist_mat);

lut_param = initial_guess.param;
distance = [];
%tic 
for j = 1: size(position_x,2)-1
    for k = (n_path-n_path_vec(j))/2 + 1:n_path_vec(j) + (n_path-n_path_vec(j))/2
        for i = (n_path-n_path_vec(j+1))/2 + 1 : n_path_vec(j+1) + (n_path-n_path_vec(j+1))/2
            %find lut entry
            %compute the startpoint and endpoint of the required path 
            startpoint = [position_x(k,j); position_y(k,j); orientation(k,j);curvature(k,j)];
            endpoint = [position_x(i, j +1 ); position_y(i,j + 1); orientation(i,j + 1);curvature(i,j +1)];
            %rotote the change in x and change in y in order to match the
            %way initial guess table was contructed
            deltay = (position_y(i,j + 1)-position_y(k,j));
            theta_rot = orientation(k,j)*-1;
            deltay = (position_x(i, j+1 )-position_x(k,j))*sin(theta_rot) + deltay*cos(theta_rot);
            %find the position in the LUT by dividing by the spacing used
            %to create LUT
            deltay = round((deltay/0.75));
            %find the change in orientation
            thetaf = (endpoint(3)- startpoint(3));
            if(abs(thetaf)>= pi)
                thetaf = thetaf + 2*pi*sign(thetaf)*-1;
            end
            %find the position in the LUT by dividing by the spacing used
            %to create LUT
            thetaf = round(thetaf/0.04) ;
            curvature_ini = round(startpoint(4)/0.01)+ 21;
            curvature_fin = round(endpoint(4)/0.01) + 21;
            if(deltay>lut_param(1))
                deltay = lut_param(1);
            elseif ((deltay<-lut_param(1)))
                deltay = -lut_param(1);
            end
        
            if(thetaf>lut_param(2))
                thetaf = lut_param(2);
            elseif ((thetaf<-(lut_param(2))))
                thetaf = -(lut_param(2));
            end
            if(curvature_ini>lut_param(3))
                curvature_ini = lut_param(3);
            elseif ((curvature_ini<1))
                curvature_ini = 1;
            end
            if(curvature_fin>lut_param(4))
                curvature_fin = lut_param(4);
            elseif ((curvature_fin<1))
                curvature_fin = 1;
            end
            %get initial guess parameters
            if(deltay<0)
                
                if(thetaf<0)
                    param = lut_neg_double((abs(deltay)-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(abs(thetaf))*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,6:10)';
                    index_lut = lut_neg_double((abs(deltay)-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(abs(thetaf))*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,11)';
                 
                else
                    param = lut_neg_lat((abs(deltay)-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(thetaf)*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,6:10)';
                    index_lut = lut_neg_lat((abs(deltay)-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(thetaf)*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,11)';
                end
            elseif(deltay == 0)
                param = [];  %should include zero in lut....
                index_lut = 0;
            elseif(thetaf<0)
                
                param =lut_neg((deltay-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(abs(thetaf))*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,6:10)';
                index_lut = lut_neg((deltay-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(abs(thetaf))*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,11)';
            else
                
                param = lut((deltay-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(thetaf)*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,6:10)';
                index_lut =lut((deltay-1)*(lut_param(2)+1)*lut_param(3)*lut_param(4)+(thetaf)*lut_param(3)*lut_param(4)+(curvature_ini-1)*lut_param(4)+ curvature_fin,11)';
            end
            
            
            %theta_ini = 0;
            
            
            if(index_lut<0.5)
                [parameters counter index] = compute_path_cubic(startpoint,endpoint,sqrt((position_x(k,j)-position_x(i, j + 1))^2+(position_y(k,j)-position_y(i, j +1))^2),param,station_new,lateral_dist*(i-k));
            else
                parameters = param;
                counter = 10;
                index = index_lut;
            end
            
            
            param_matrix(i,(j-1)*7 +1 :(j-1)*7 +7, k) = [parameters' counter index];
            kinematics = (sum(abs(parameters(1:4))>max_curv))*1000000000000;
            %%%%%% what this condition meand
            if(draw)&&(index<0.5)&&(kinematics<1)
                if(parameters(end)>12.3)
                    checking = 1;
                end
                [x y theta1 curvature1 spacing]= drawtrajec_abcd(parameters,[position_x(k,j); position_y(k,j); orientation(k,j);curvature(k,j)],[position_x(1, j +1 ); position_y(1,j + 1); orientation(1,j + 1);curvature(1,j +1)],20);
              
                distance = [distance; parameters(end)];
                hold on
                h = plot(x,y,'LineWidth',1.5);
                % axis([-10 10 -10 250]);
                axis equal
                set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            end
        end
    end
end

%once the lattice is complete it needs to be merged with the starting pose
%by using a quartic curvature polynomial

%toc

for i = (n_path-n_path_vec(1))/2 + 1:n_path_vec(1) + (n_path-n_path_vec(1))/2
    
    endpoint = [position_x(i, 1 ); position_y(i,1); orientation(i,1);curvature(i, 1)];
    deltax = (endpoint(1)-start_pose(1));
    deltay = (position_y(i,1)-start_pose(2));
    theta_rot = start_pose(3)*-1;
    deltay1 = (deltax)*sin(theta_rot) + deltay*cos(theta_rot);
    deltax1 = -(deltay)*sin(theta_rot) + deltax*cos(theta_rot);
    thetaf = (endpoint(3)- start_pose(3));
    if(abs(thetaf)>= pi)
        thetaf = thetaf + 2*pi*sign(thetaf)*-1;
    end
    %normalise start point so that all initial states are zero apart from
    %curvature and curvature prime
    start_pose_norm = [ 0,0,0,start_pose(4),start_pose(5)];
    endpoint_norm = [deltax1,deltay1,thetaf,endpoint(4)];
    [parameters, counter, index] = compute_path_quartic(start_pose_norm,endpoint_norm);
    param_matrix_start(i,:) = [parameters' counter index];
     kinematics = (sum(abs(parameters(1:5))>max_curv))*1000000000000;
    if(draw)&&(index<1)&&(kinematics<1)
    [x, y]= drawtrajec_abcde(parameters,[start_pose(1:4)],endpoint,20);
    hold on
    h = plot(x,y,'-g','LineWidth',1.5);
    set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off')

    end
    if(draw)&&(index<1)&&(kinematics>1)
    [x, y]= drawtrajec_abcde(parameters,[start_pose(1:4)],endpoint,20);
    hold on
    h = plot(x,y,'-*k','LineWidth',1.5);
    set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off')
    end
end

%% define the pose of the obstacle 1
car_pose = [ 3 23 90; -3 55 90];% -1.5 55 90];
% draw the car obstacle 
for i = 1:length(car_pose(:,1))
    draw_car_obstacle(dimension, car_pose(i,:));
end


%define minimum velocity
vmin = 2.8; % 2.8m/s equivalent to 10km/hr
%vmax = 16.7;
vmax = 13.89;
%velocity discretisation level
n_vel = 5;
v_vec = vmin*ones(n_vel,1)+ (0:(vmax-vmin)/(n_vel-1):(vmax-vmin))';




start_car.start = start_pose ;
start_car.traj= param_matrix_start ;
start_car.dimension = dimension;
lattice.x = position_x;
lattice.y = position_y ;
lattice.theta = orientation ;
lattice.curve = curvature ;
lattice.param = param_matrix;
lattice.vel = v_vec;
lattice.sample = sub_sample;

%draw_car(dimension,[start_pose(1) start_pose(2) start_pose(3)/pi*180])
%[ optimal_par] = dynamic07082015(param_matrix_start,start_pose,param_matrix,v_vec,centre_line,dimension, sub_sample,position_x,position_y,orientation, n_path_vec);
[ optimal_par] = dynamic_eff(param_matrix_start,start_pose,param_matrix,v_vec,dimension, sub_sample,position_x,position_y,orientation, n_path_vec);
no_coll = check_path(optimal_par, static_obstacle_matrix, lattice, start_car, car_pose);
if (no_coll > 0)
    [ optimal_par] = dynamic_obs(param_matrix_start, start_pose, param_matrix, v_vec, dimension, sub_sample, position_x, position_y, orientation, n_path_vec, static_obstacle_matrix,car_pose);
end
 
draw_path = 1;
draw_car_flag = 0;
draw_optimal_path(optimal_par,start_car,lattice, draw_car_flag, draw_path)








end


function [x y theta curvature spacing]= drawtrajec(param,startpoint,endpoint,step)
spacing = 0:param(end)/step:param(end);
theta = repmat(startpoint(3),1,length(spacing)) + cubic_theta(param,spacing,1);
curvature = curvature_comp(param,spacing,1);
%j = 1;
i = param(end)/step:param(end)/step:param(end);
%j = j+1;
x = integral_par_group(0,[param(1:4)], i,0,1) + startpoint(1);
y =  integral_par_group(0,[param(1:4)], i,1,1) + startpoint(2);

x = [startpoint(1) ;x];
y = [startpoint(2); y];
end



function [param counter delta_state_index] = compute_path_cubic(startpoint,endpoint,distance,initial,station_new, lat_dist)




% path model


%  a) this is done using a curvature polynomial with unknown parameters->quartic to connect current pose to endpoints, cubic to connect them between themselves
%  b)gradient descent method to find parameters.**whole subproblem in itself


%calculate the jacobian of the state equation for a curvature = a + bs
%+cs^2+ ds^3
%parameters for endpoints a b c d sf

%jacobian = [dx/dq ; dy/dq;dtheta/dq;dcurvature/dq];


%simpsons rule
scale = distance;

thetaf = (endpoint(3)- startpoint(3));
theta_ini = startpoint(3);

%find the real difference between the two angles if it is more than 180
%degrees it means that one of the angles is not in the right angle representation
if(abs(thetaf)>= pi)
    thetaf = thetaf + 2*pi*sign(thetaf)*-1;
end
%if no initial guess parameters are given start with a straight line
if(isempty(initial))
    %initialisation
    param_old = ones(5,1);
    
    kf = endpoint(4);
    k0 = startpoint(end);

    param_old(end) = 0.25*station_new + sqrt((0.75*station_new)^2+(lat_dist)^2);
    sf = param_old(end);
    %finding b and c set d to zero
    d = 0;
    %param_old(2:3,1) = [sf/scale^2 sf^2/scale^3 ; sf^2/(2*scale^2) sf^3/(scale^3*3)]\[kf-k0/scale-d*sf^3/scale^4; thetaf-k0*sf/scale-d*sf^4/(scale^4*4)];% A\b = inv(A)*b
    param_b_c = [sf sf^2 ; sf^2/(2) sf^3/(3)]\[kf-k0-d*sf^3; thetaf-k0*sf-d*sf^4/(4)];% A\b = inv(A)*b
    param_old(2) = k0 + param_b_c(1)*param_old(end)/3 + param_b_c(2)*param_old(end)^2/9;
    param_old(3) = k0 + param_b_c(1)*2*param_old(end)/3 + param_b_c(2)*4*param_old(end)^2/9;
    
  
    param_old(1)= k0;
    param_old(4) = kf;
else
    param_old =[startpoint(end);initial(2:3);endpoint(4);initial(end)];
    % param_old =initial
end

%iterate to find the correct parameters

tolerance = 0.025;

weights = [ 1; 1; 100];

scale = 1;

%converting between parameters from p1, p2, p3,p4 to a b c d
a = param_old(1);
b = -(11*param_old(1) -18*param_old(2) +9*param_old(3) -2*param_old(4))/(2*param_old(end));
c = 9*(2*param_old(1) - 5*param_old(2) + 4*param_old(3) - param_old(4))/(2*param_old(end)^2);
d = -9*(param_old(1) -3*param_old(2) + 3*param_old(3) -param_old(4))/(2*param_old(end)^3);
param_abcd = [ a ; b;c;d];

%only 3 parameters need to be found therefore delta state should be made of
%3 components
delta_state = [integral_par(0,[param_abcd; param_old(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param_old(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param_old(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf)];
delta_state_index = sqrt(sum((weights.*delta_state).^2));
counter = 0;
param = param_old;
damping = 0.0001;

while(delta_state_index> tolerance)&&(counter<10)
    counter = counter + 1;
    
    delta_param = -( diff_par_same_scale(param_old,param_old(end), endpoint,thetaf, theta_ini)+damping*eye(3))\delta_state;
    %update parameters
    param = param_old + [0;delta_param(1:2);0;delta_param(end)];
    
    a=param(1);
    b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
    c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
    d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
    param_abcd = [ a ; b;c;d];
    %compute difference from current state to goal state
    delta_state = [integral_par(0,[param_abcd; param(end)],0,scale, theta_ini); integral_par(0,[param_abcd; param(end)],1,scale, theta_ini) ; cubic_theta(param_abcd,param(end),scale, theta_ini)] + [startpoint(1:2)- endpoint(1:2);-(theta_ini+thetaf)];
    
    %compute performance index
    delta_state_index = sqrt(sum((weights.*delta_state).^2));
    
    param_old = param;
end

end

function jacobian_par = diff_par(par_int, s,scale, endpoint, thetaf, theta_ini)
%differentiating also wrt to a
% dxdq = [ -integral_par(1,par_int,1) -integral_par(2,par_int,1)/2 -integral_par(3,par_int,1)/3 -integral_par(4,par_int,1)/4 cos(cubic_theta(par_int,s))];
% dydq = [-integral_par(1,par_int,0) -integral_par(2,par_int,0)/2 -integral_par(3,par_int,0)/3 -integral_par(4,par_int,0)/4 sin(cubic_theta(par_int,s))];
% dthetadq =[s (s^2)/2 (s^3)/3 (s^4)/4 curvature_comp(par_int,s) ] ;
% dcurvaturedq = [1 s (s^2) (s^3) curvature_prime_comp(par_int)] ;




dxdq = [ -integral_par(2,par_int,1,scale,theta_ini)/2 -integral_par(3,par_int,1,scale, theta_ini)/3 -integral_par(4,par_int,1,scale, theta_ini)/4 cos(thetaf + theta_ini)];
dydq = [integral_par(2,par_int,0,scale, theta_ini)/2 integral_par(3,par_int,0,scale, theta_ini)/3 integral_par(4,par_int,0,scale, theta_ini)/4 sin(thetaf + theta_ini)];
dthetadq =[(s^2)/(scale^2*2) (s^3)/(scale^3*3) (s^4)/(scale^4*4) endpoint(end) ] ;
dcurvaturedq = [s/scale^2 (s^2)/scale^3 (s^3)/scale^4 curvature_prime_comp(par_int,s,scale)] ;
jacobian_par = [dxdq ;dydq ;dthetadq ;dcurvaturedq];
end

%par = [p0 p1 p2 p3 p4 sf]










