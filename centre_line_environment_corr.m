function [static_obstacle_matrix, line_matrix_out ,centre_step_fine] =  centre_line_environment_corr(road_config, start,final)



total_dist = final.distance - start.distance; 

centre_step_fine= single(total_dist/round(total_dist/0.1));

steps  = total_dist/ centre_step_fine;
% discretisation of the obstacles
line_obs_x_left = repmat(-road_config.lane_width/2,1,steps);
line_obs_x_right = repmat(road_config.lane_width/2,1,steps);
line_obs_y_left = zeros(1,steps);
line_obs_y_right = zeros(1,steps);
line_obs_y_left(1) = start.distance;
line_obs_y_right(1) = start.distance;
for i = 1: steps - 1
    line_obs_y_left(i + 1) = line_obs_y_left(i) + centre_step_fine;

    line_obs_y_right(i + 1) = line_obs_y_right(i) + centre_step_fine;
end

static_obstacle_matrix = [line_obs_x_left;line_obs_y_left;line_obs_x_right;line_obs_y_right];



%% dummy code test for the obstacles
for i = 1:size(static_obstacle_matrix,2)
    if (static_obstacle_matrix(2,i) >= 30)&& (static_obstacle_matrix(2,i) <= 35)
        static_obstacle_matrix(1,i) = -2;
    end
    if (static_obstacle_matrix(2,i) >= 45)&& (static_obstacle_matrix(2,i) <= 50)
        static_obstacle_matrix(3,i) = 2;
    end
end


% because it is straight line y for left and right are the same
line_obs_y = line_obs_y_left; % later can be changed
line_matrix = [line_obs_x_left + line_obs_x_right; line_obs_y] ;


x_prime = single(central_diff_1(single(line_matrix(1,:)),centre_step_fine));
y_prime = single(central_diff_1(single(line_matrix(2,:)),centre_step_fine));
x_primec = gradient(line_matrix(1,:),centre_step_fine);
y_primec = gradient(line_matrix(2,:),centre_step_fine);
x_d_prime = single(central_diff_2(single(line_matrix(1,:)),centre_step_fine));
y_d_prime = single(central_diff_2(single(line_matrix(2,:)),centre_step_fine));

orientation = atan2(y_prime,x_prime);
curvature = single(central_diff_1(single(orientation),centre_step_fine));


line_matrix_out = [line_matrix; orientation; curvature];




end