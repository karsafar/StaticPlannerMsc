function [ optimal_path] = dynamic_eff(param_matrix_start,start_pose, full_set,velocities, dimension,sub_sample,position_x,position_y,theta_ini,n_path_vec)
max_travel = inf;
no_more = 0;
max_curv =1/3.8;
n_path = size(full_set,1);

x = zeros(1,sub_sample+1);
y = zeros(1,sub_sample+1);

% set the costs for the first part of the lattice
cost_ini = zeros(n_path,1,length(velocities));
backtrack_cost = [Inf(n_path,length(full_set)/7 + 1,length(velocities))];
%backtrack_cost = [cost_ini Inf(n_path,length(full_set)/7 ,length(velocities))];
backtrack_state= [ Inf(n_path,length(full_set)/7 +1 ,length(velocities),2)];


feas_check = zeros(1,n_path);
%comp_check = zeros(1,n_path);
% feas_check_vel = zeros(n_path,length(velocities));
% comp_check_vel = cell(n_path,1);
number_array = 1:n_path;
%number_array_vel = repmat(1:length(velocities),n_path,1);
error = 0;
% you need to enforce vini for initial pose
vel = start_pose(6);
for next_lateral = (n_path-n_path_vec(1))/2 + 1 : n_path_vec(1) + (n_path-n_path_vec(1))/2
    if(param_matrix_start(next_lateral,8)< 0.5)&&((sum(abs(param_matrix_start(next_lateral,3:5))>max_curv))==0)&&(param_matrix_start(next_lateral,6)<2*sqrt((start_pose(1)-position_x(next_lateral,1))^2 + (start_pose(2)-position_y(next_lateral,1))^2))
        param = param_matrix_start(next_lateral,1:6);
        a = param(1);
        b = param(2);
        c = -(85*param(1) - 108*param(3) + 27*param(4) - 4*param(5) + 22*param(2)*param(end))/(4*param(end)^2);
        d = (9*(10*param(1) - 15*param(3) + 6*param(4) - param(5) + 2*param(2)*param(end)))/(2*param(end)^3);
        
        e = -(9*(11*param(1) - 18*param(3) + 9*param(4) - 2*param(5) + 2*param(2)*param(end)))/(4*param(end)^4);
        
        
        
        for next_vel = 1:length(velocities)
            
            
            
            
            [s,T, acc, jerk, vel_prof] = velgen(vel,velocities(next_vel),start_pose(7),0,param(end),sub_sample);
            %traj point format [x;y;theta;curvature;curvature';speed;acceleration;jerk; time]
            %                         for position=1: length(s)-1
            %                             x(position+1)= integral_par(0,[a;b;c;d; s(position +1)],0,1, theta_ini(lateral,station)) + position_x(lateral,station);
            %                             y(position+1) =integral_par(0,[a;b;c;d;s(position +1)],1,1, theta_ini(lateral,station))+ position_y(lateral,station);
            %                         end
            %[s,T, acc, jerk, vel_prof] = velgen(vel,velocities(next_vel),start_pose(7),0,param(end),sub_sample);
            
            x(1) = start_pose(1);
            y(1) = start_pose(2);
            if(isempty(s(2:end)))
                stoppp = 1;
            end
            x(2:end)= integral_par_group_quart(0,[a;b;c;d;e],s(2:end),0,1, start_pose(3)) + x(1);
            y(2:end)= integral_par_group_quart(0,[a;b;c;d;e],s(2:end),1,1, start_pose(3))+ y(1);
            theta = quartic_theta([a;b;c;d;e],s,1,start_pose(3));
            curvature=curvature_comp_quart([a;b;c;d;e],s,1);
            curvature_dot= curvature_dot_comp_quart([a;b;c;d;e],s,1,vel_prof);
            sample_traj_points = [x; y ;theta;curvature;curvature_dot; vel_prof;acc; jerk; T*ones(1,length(jerk))];
            %trajectory_cost = total_cost(sample_traj_points,centre_line_seg,param(end), dimension);
            children =full_set(:,1:7,next_lateral);
            
            
            %trajectory_cost= total_cost_r(sample_traj_points,centre_line_seg,param(end), dimension,children);
            trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children);
            if (backtrack_cost(next_lateral,1,next_vel)> 0 + trajectory_cost)
                backtrack_cost(next_lateral,1,next_vel)= 0+ trajectory_cost;
                
                backtrack_state(next_lateral,1,next_vel,:) =[1 vel];
                
                error =1;
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

%feas_check = zeros(1,n_path);
if(error ==0)
    problem = 1;
end

%vel_check = zeros(size(backtrack,2),1);

for station = 1: length(full_set)/7
    
    
    
    
    for lateral = (n_path-n_path_vec(station))/2 + 1:n_path_vec(station) + (n_path-n_path_vec(station))/2
        if(sum(lateral == comp_check)==0)
            
            for next_lateral = (n_path-n_path_vec(station+1))/2 + 1 : n_path_vec(station+1) + (n_path-n_path_vec(station+1))/2
                if(full_set(next_lateral,(station-1)*7 +7,lateral)< 0.5)&&((sum(abs(full_set(next_lateral,(station-1)*7 +1:(station-1)*7 +4,lateral))>max_curv))==0)&&(full_set(next_lateral,(station-1)*7 +5,lateral)<2*sqrt((position_x(lateral,station)-position_x(next_lateral,station+1))^2 + (position_y(lateral,station)-position_y(next_lateral,station+1))^2))
                    param = full_set(next_lateral,(station-1)*7 +1:(station-1)*7 +5,lateral);
                    a=param(1);
                    b = -(11*param(1) -18*param(2) +9*param(3) -2*param(4))/(2*param(end));
                    c = 9*(2*param(1) - 5*param(2) + 4*param(3) - param(4))/(2*param(end)^2);
                    d = -9*(param(1) -3*param(2) + 3*param(3) -param(4))/(2*param(end)^3);
                    
                    for vel = 1:length(velocities)
                       % if(sum(vel == comp_check_vel{lateral})==0)
                           % for next_vel = 1:length(velocities)
                             for next_vel = max([vel-2 1]):min([vel+2 length(velocities)])   
                                
                                
                                
                                [s,T, acc, jerk, vel_prof] = velgen(velocities(vel),velocities(next_vel),0,0,param(5),sub_sample);
                                %traj point format [x;y;theta;curvature;curvature';speed;acceleration;jerk; time]
                                %                         for position=1: length(s)-1
                                %                             x(position+1)= integral_par(0,[a;b;c;d; s(position +1)],0,1, theta_ini(lateral,station)) + position_x(lateral,station);
                                %                             y(position+1) =integral_par(0,[a;b;c;d;s(position +1)],1,1, theta_ini(lateral,station))+ position_y(lateral,station);
                                %                         end
                                x(1) = position_x(lateral,station);
                                y(1) = position_y(lateral,station);
                                if(isempty(s(2:end)))
                                    stoppp = 1;
                                end
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
                                
                                %trajectory_cost= total_cost_r(sample_traj_points,centre_line_seg,param(end), dimension,children);
                                trajectory_cost= total_cost_s(sample_traj_points,param(end), dimension,children);
                                
                                if (backtrack_cost(next_lateral,station+1,next_vel)> backtrack_cost(lateral,station,vel) + trajectory_cost)
                                    backtrack_cost(next_lateral,station+1,next_vel)= backtrack_cost(lateral,station,vel)+ trajectory_cost;
                                    
                                    backtrack_state(next_lateral,station+1,next_vel,:) =[lateral vel];
                                    
                                    
                                end
                            end
                       % end
                    end
                end
                %                 if(sum(isinf(backtrack_cost(next_lateral,station+1,:))|backtrack_cost(next_lateral,station+1,:)>=1000000000000) ==length(velocities))
                %                     feas_check(next_lateral)=  feas_check(next_lateral)+1;
                %                 end
            end
        end
    end
    
    
    feas_check= sum(isinf(backtrack_cost(:,station+1,:))|backtrack_cost(:,station+1,:)>=1000000000000,3)==length(velocities);
    
    comp_check = number_array(logical(feas_check));
    if(sum(feas_check)==n_path)&&(no_more == 0)
        no_more = 1;
        max_travel = station;
    end
%      comp_check_not = number_array(~logical(feas_check));
% 
%     feas_check_vel(comp_check_not,:) = reshape(isinf(backtrack_cost(comp_check_not,station+1,:))|backtrack_cost(comp_check_not,station+1,:)>=1000000000000,length(comp_check_not),length(velocities));
%     for loop = 1: n_path
%     comp_check_vel(loop) = num2cell(number_array(logical(feas_check_vel(loop,:))),2);
%     end
end

%find optimal
tavelled_dist =min([length(full_set)/7 max_travel-1]);
optimal_path = zeros(3,tavelled_dist +2 );


%[optimal_path(1,tavelled_dist +2),index(2)] = min(min(backtrack_cost(:,end,:)));%cost
[optimal_path(1,tavelled_dist +2),index(2)] = min(min(backtrack_cost(:,tavelled_dist + 1,:)));%cost

%[temp, index_temp] =(min(backtrack_cost(:,end,:)));%cost
[temp, index_temp] =(min(backtrack_cost(:,tavelled_dist + 1,:)));%cost

index(1) = index_temp(index(2));
optimal_path(2,tavelled_dist +2) = index(1);%current node
optimal_path(3,tavelled_dist +2) = index(2);%current vel

% for search =  size(backtrack_cost,2):-1:2
%
%     optimal_path(2,search-1) = backtrack_state(index(1),search,index(2),1); %prev_node
%     optimal_path(3,search-1) = backtrack_state(index(1),search,index(2),2);%current_vel_vf
%     optimal_path(1,search-1) = backtrack_cost( optimal_path(2,search-1),search-1,optimal_path(3,search-1));%total_cost
%     index(1) =  optimal_path(2,search-1);
%     index(2) = optimal_path(3,search-1);
% end


for search =  tavelled_dist + 1:-1:1
    
    optimal_path(2,search) = backtrack_state(index(1),search,index(2),1); %prev_node
    optimal_path(3,search) = backtrack_state(index(1),search,index(2),2);%current_vel_vf
    if(search == 1)
        optimal_path(1,search) = 0;
    else
        if(isinf(optimal_path(2,search)))
            stop_now = 1;
        end
        
        
        optimal_path(1,search) = backtrack_cost( optimal_path(2,search),search-1,optimal_path(3,search));%total_cost
    end
    index(1) =  optimal_path(2,search);
    index(2) = optimal_path(3,search);
end

end