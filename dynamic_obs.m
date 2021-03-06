function [ optimal_path] = dynamic_obs(param_matrix_start,start_pose, full_set,velocities, dimension,sub_sample,position_x,position_y,theta_ini,n_path_vec,obstacles, car_pose)
max_curv =1/3.8;
n_path = size(full_set,1);

x = zeros(1,sub_sample+1);
y = zeros(1,sub_sample+1);

% set the costs for the first part of the lattice
cost_ini = zeros(n_path,1,length(velocities));
backtrack_cost = [Inf(n_path,length(full_set)/7 + 1,length(velocities))];
%backtrack_cost = [cost_ini Inf(n_path,length(full_set)/7 ,length(velocities))];
backtrack_state= [ Inf(n_path,length(full_set)/7 +1 ,length(velocities),2)];


% you need to enforce vini for initial pose


feas_check = zeros(1,n_path);
%comp_check = zeros(1,n_path);
% feas_check_vel = zeros(n_path,length(velocities));
% comp_check_vel = cell(n_path,1);
number_array = 1:n_path;
vel = start_pose(6);
for next_lateral = (n_path-n_path_vec(1))/2 + 1 : n_path_vec(1) + (n_path-n_path_vec(1))/2
    if(param_matrix_start(next_lateral,8)< 0.5)&&((sum(abs(param_matrix_start(next_lateral,3:5))>max_curv))==0)
        param = param_matrix_start(next_lateral,1:6);
        a = param(1);
        b = param(2);
        c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);
        d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);
        
        e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);
        x(1) = start_pose(1);
        y(1) = start_pose(2);
        x(2:end)= integral_par_group_quart(0,[a;b;c;d;e],param(end)/sub_sample:param(end)/sub_sample:param(end),0,1, start_pose(3)) + x(1);
        y(2:end)= integral_par_group_quart(0,[a;b;c;d;e],param(end)/sub_sample:param(end)/sub_sample:param(end),1,1, start_pose(3))+ y(1);
        theta = quartic_theta([a;b;c;d;e],0:param(end)/sub_sample:param(end),1,start_pose(3));
        %         [no_of_coll, positions] = collision_detection(obstacles(:,1:sub_sample+1),[x;y;theta], dimension);
        %% my DUMMY static obstacle code
        [no_of_coll, positions] = static_obstacles(obstacles(:,1:sub_sample+1),[x;y;theta], dimension, car_pose);
        if(no_of_coll==0)
            for next_vel = 1:length(velocities)
                
                
                
                
                [s,T, acc, jerk, vel_prof] = velgen(vel,velocities(next_vel),start_pose(7),0,param(end),sub_sample);
                %traj point format [x;y;theta;curvature;curvature';speed;acceleration;jerk; time]
                %                         for position=1: length(s)-1
                %                             x(position+1)= integral_par(0,[a;b;c;d; s(position +1)],0,1, theta_ini(lateral,station)) + position_x(lateral,station);
                %                             y(position+1) =integral_par(0,[a;b;c;d;s(position +1)],1,1, theta_ini(lateral,station))+ position_y(lateral,station);
                %                         end
                x(1) = start_pose(1);
                y(1) = start_pose(2);
                x(2:end) = integral_par_group_quart(0,[a;b;c;d;e],s(2:end),0,1, start_pose(3)) + x(1);
                y(2:end) = integral_par_group_quart(0,[a;b;c;d;e],s(2:end),1,1, start_pose(3))+ y(1);
                theta = quartic_theta([a;b;c;d;e],s,1,start_pose(3));
                curvature = curvature_comp_quart([a;b;c;d;e],s,1);
                curvature_dot = curvature_dot_comp_quart([a;b;c;d;e],s,1,vel_prof);
                sample_traj_points = [x; y ;theta;curvature;curvature_dot; vel_prof;acc; jerk; T*ones(1,length(jerk))];
                %trajectory_cost = total_cost(sample_traj_points,centre_line_seg,param(end), dimension);
                children = full_set(:,1:7,next_lateral);
                
                
                %trajectory_cost= total_cost_r1(sample_traj_points,centre_line_seg,param(end), dimension,children);
                if next_lateral < 5
                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 500;
                elseif next_lateral == 5
                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 300;
                elseif next_lateral == 6
                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 200;
                elseif next_lateral == 7
                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children);
                end
                
                
                if (backtrack_cost(next_lateral,1,next_vel))>(0 + trajectory_cost)
                    backtrack_cost(next_lateral,1,next_vel) = 0+ trajectory_cost;
                    
                    backtrack_state(next_lateral,1,next_vel,:) = [1 vel];
                    
                    
                end
            end
        end
    end
    if(sum(isinf(backtrack_cost(next_lateral,1,:))|backtrack_cost(next_lateral,1,:)>=1000000000000) ==length(velocities))
        feas_check(next_lateral)=  feas_check(next_lateral)+1;
        %     else
        %         feas_check_vel(next_lateral,:) = reshape(isinf(backtrack_cost(next_lateral,1,:))|backtrack_cost(next_lateral,1,:)>=1000000000000,1,length(velocities));
        %         comp_check_vel(next_lateral) = num2cell(number_array(logical(feas_check_vel(next_lateral,:))),2);
    end
end






comp_check = number_array(logical(feas_check));
%vel_check = zeros(size(backtrack,2),1);

for station = 1: length(full_set)/7
    
    
    
    
    
    for lateral = (n_path-n_path_vec(station))/2 + 1:n_path_vec(station) + (n_path-n_path_vec(station))/2
        if(sum(lateral == comp_check)==0)
            
            for next_lateral = (n_path-n_path_vec(station+1))/2 + 1 : n_path_vec(station+1) + (n_path-n_path_vec(station+1))/2
                if(full_set(next_lateral,(station-1)*7 +7,lateral) < 0.5)&&((sum(abs(full_set(next_lateral,(station-1)*7 +1:(station-1)*7 +4,lateral))>max_curv))==0)
                    param = full_set(next_lateral,(station-1)*7 +1:(station-1)*7 +5,lateral);
                    a = param(1);
                    b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    
                    x(1) = position_x(lateral,station);
                    y(1) = position_y(lateral,station);
                    x(2:end)= integral_par_group(0,[a;b;c;d],param(end)/sub_sample:param(end)/sub_sample:param(end),0,1, theta_ini(lateral,station)) + position_x(lateral,station);
                    y(2:end)= integral_par_group(0,[a;b;c;d],param(end)/sub_sample:param(end)/sub_sample:param(end),1,1, theta_ini(lateral,station))+ position_y(lateral,station);
                    theta = cubic_theta([a;b;c;d],0:param(end)/sub_sample:param(end),1,theta_ini(lateral,station));
                    if (station == length(full_set)/7)% if the last section
                        %                          [no_of_coll, positions] = collision_detection(obstacles(:,(station)*sub_sample+1:end),[x;y;theta], dimension);
                        
                        %% my DUMMY static obstacle code
                        [no_of_coll, positions] = static_obstacles(obstacles(:,(station)*sub_sample+1:end),[x;y;theta], dimension, car_pose);
                        
                    else % if not the last section
                        %                          [no_of_coll, positions] = collision_detection(obstacles(:,(station)*sub_sample+1:(station)*sub_sample+1 +sub_sample +1),[x;y;theta], dimension);
                        
                        %% my DUMMY static obstacle code
                        [no_of_coll, positions] = static_obstacles(obstacles(:,(station)*sub_sample+1:(station)*sub_sample+1 +sub_sample +1),[x;y;theta], dimension,car_pose);
                        
                    end
                    if(no_of_coll==0)
                        for vel = 1:length(velocities)
                            %for next_vel = 1:length(velocities)
                            for next_vel = max([vel-2 1]):min([vel+2 length(velocities)])
                                [s,T, acc, jerk, vel_prof] = velgen(velocities(vel),velocities(next_vel),0,0,param(5),sub_sample);
                                %traj point format [x;y;theta;curvature;curvature';speed;acceleration;jerk; time]
                                %                         for position=1: length(s)-1
                                %                             x(position+1)= integral_par(0,[a;b;c;d; s(position +1)],0,1, theta_ini(lateral,station)) + position_x(lateral,station);
                                %                             y(position+1) =integral_par(0,[a;b;c;d;s(position +1)],1,1, theta_ini(lateral,station))+ position_y(lateral,station);
                                %                         end
                                x(1) = position_x(lateral,station);
                                y(1) = position_y(lateral,station);
                                x(2:end)= integral_par_group(0,[a;b;c;d],s(2:end),0,1, theta_ini(lateral,station)) + position_x(lateral,station);
                                y(2:end)= integral_par_group(0,[a;b;c;d],s(2:end),1,1, theta_ini(lateral,station))+ position_y(lateral,station);
                                theta = cubic_theta([a;b;c;d],s,1,theta_ini(lateral,station));
                                curvature=curvature_comp([a;b;c;d],s,1);
                                curvature_dot= curvature_dot_comp([a;b;c;d],s,1,vel_prof);
                                sample_traj_points = [x; y ;theta;curvature;curvature_dot; vel_prof;acc; jerk; T*ones(1,length(jerk))];
                                %trajectory_cost = total_cost(sample_traj_points,centre_line_seg,param(end), dimension);
                                if(station ==  length(full_set)/7)
                                    children =[];
                                else
                                    children =full_set(:,(station)*7 +1:(station)*7 +7,next_lateral);
                                end
                                
                                %trajectory_cost= total_cost_r1(sample_traj_points,centre_line_seg,param(end), dimension,children);
                                if next_lateral < 5
                                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 500;
                                elseif next_lateral == 5
                                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 300;
                                elseif next_lateral == 6
                                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children) + 200;
                                elseif next_lateral == 7
                                    trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children);
                                end
                                
                                if (backtrack_cost(next_lateral,station+1,next_vel))> (backtrack_cost(lateral,station,vel) + trajectory_cost)
                                    backtrack_cost(next_lateral,station+1,next_vel)= backtrack_cost(lateral,station,vel)+ trajectory_cost;
                                    
                                    backtrack_state(next_lateral,station+1,next_vel,:) =[lateral vel];
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    
    
    feas_check= sum(isinf(backtrack_cost(:,station+1,:))|backtrack_cost(:,station+1,:)>=1000000000000,3)==length(velocities);
    
    comp_check = number_array(logical(feas_check));
end
%find optimal
optimal_path = zeros(3,length(full_set)/7 +2 );


[optimal_path(1,length(full_set)/7 +2),index(2)] = min(min(backtrack_cost(:,end,:)));%cost
if(isinf(optimal_path(1,end)))
    dec = 1;
    while( isinf(min(min(backtrack_cost(:,end-dec,:)))))
        [optimal_path(1,length(full_set)/7 +2-dec),index(2)] = min(min(backtrack_cost(:,end-dec,:)));%cost
        
        dec = dec + 1;
    end
    [optimal_path(1,length(full_set)/7 +2-dec),index(2)] = min(min(backtrack_cost(:,end-dec,:)));%cost
    
    [temp, index_temp] =(min(backtrack_cost(:,end-dec,:)));%cost
    index(1) = index_temp(index(2));
    optimal_path(2,length(full_set)/7 +2-dec) = index(1);%current node
    optimal_path(3,length(full_set)/7 +2-dec) = index(2);%current vel
    
    % for search =  size(backtrack_cost,2):-1:2
    %
    %     optimal_path(2,search-1) = backtrack_state(index(1),search,index(2),1); %prev_node
    %     optimal_path(3,search-1) = backtrack_state(index(1),search,index(2),2);%current_vel_vf
    %     optimal_path(1,search-1) = backtrack_cost( optimal_path(2,search-1),search-1,optimal_path(3,search-1));%total_cost
    %     index(1) =  optimal_path(2,search-1);
    %     index(2) = optimal_path(3,search-1);
    % end
    
    
    for search =  size(backtrack_cost,2)-dec:-1:1
        
        optimal_path(2,search) = backtrack_state(index(1),search,index(2),1); %prev_node
        optimal_path(3,search) = backtrack_state(index(1),search,index(2),2);%current_vel_vf
        if(search == 1)
            optimal_path(1,search) = 0;
        else
            optimal_path(1,search) = backtrack_cost( optimal_path(2,search),search-1,optimal_path(3,search));%total_cost
        end
        index(1) =  optimal_path(2,search);
        index(2) = optimal_path(3,search);
    end
else
    
    [temp, index_temp] =(min(backtrack_cost(:,end,:)));%cost
    index(1) = index_temp(index(2));
    optimal_path(2,length(full_set)/7 +2) = index(1);%current node
    optimal_path(3,length(full_set)/7 +2) = index(2);%current vel
    
    % for search =  size(backtrack_cost,2):-1:2
    %
    %     optimal_path(2,search-1) = backtrack_state(index(1),search,index(2),1); %prev_node
    %     optimal_path(3,search-1) = backtrack_state(index(1),search,index(2),2);%current_vel_vf
    %     optimal_path(1,search-1) = backtrack_cost( optimal_path(2,search-1),search-1,optimal_path(3,search-1));%total_cost
    %     index(1) =  optimal_path(2,search-1);
    %     index(2) = optimal_path(3,search-1);
    % end
    
    
    for search =  size(backtrack_cost,2):-1:1
        
        optimal_path(2,search) = backtrack_state(index(1),search,index(2),1); %prev_node
        optimal_path(3,search) = backtrack_state(index(1),search,index(2),2); %current_vel_vf
        if(search == 1)
            optimal_path(1,search) = 0;
        else
            optimal_path(1,search) = backtrack_cost( optimal_path(2,search),search-1,optimal_path(3,search));%total_cost
        end
        index(1) =  optimal_path(2,search);
        index(2) = optimal_path(3,search);
    end
end
end