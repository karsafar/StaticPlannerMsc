function [result_table] = testbench_planner_new()
%clear all
close all

load('lut_neg_med_lat.mat','lut_neg_med_lat');
load('lut_neg_med.mat','lut_neg_med');
load('lut_pos_med.mat','lut_pos_med');
load('lut_double_neg_med.mat','lut_double_neg_med');
initial_guess_table.neg_lat = lut_neg_med_lat;
initial_guess_table.neg_ang = lut_neg_med;
initial_guess_table.pos = lut_pos_med;
initial_guess_table.neg_double = lut_double_neg_med;
initial_guess_table.param = [10 26 41 41];
%table where to store results
result_table= inf(44,4);%44%77

%% prior road configurations
road.lane_numbers = 1; % ask if it is a lane number or numbers???
road.lane_width = 8;
%area= 60;   why would I need this parameter???


%vehicle start pose
start_pose = [ 0; 4;120/180*pi; 0; 0 ; 8;0];  % 
% separation between sampled states
station = 10;

start.distance = 10; % distance from the end of the road???

final.distance = 200; % probably distance from the end of the road too.. ???


%distance to simulate centreline and roundabout features after exit start
%location
%final.distance = 15;%50

%construct the prior centreline and obstacle map which are the roundabout curb using
%the given roundabout specifications


%% create a centreline for the road
[static_obstacle_matrix, line_matrix ,stepfine] = centre_line_environment_corr(road, start, final );


%plot centreline
hold on
plot(line_matrix(1,:),line_matrix(2,:),'-r*');
hold on
xlabel('x(m)','FontSize',14)
ylabel('y(m)','FontSize',14)
title('Road','FontSize',16)
%typical bowler dimension http://www.the-blueprints.com/blueprints/cars/variouscars/40748/view/bowler_nemesis/

width_car = 2.16;
length_car = 4.4;
% %len is the disance between the front and the back wheels
len = 2.75;
%
dimension = [width_car length_car len];

%% need to be corrected to my code


%subsample the prior line
sub_sample = round(station/stepfine);
station_new = stepfine* sub_sample; % this is the same for the straight line as station
sub_centre_line = line_matrix(:,1:sub_sample:end);


%create prior belief

lateral_dist = 0.75;
%prior_uncertainty = calc_prior_belief(sub_centre_line);
prior_uncertainty = sub_centre_line;


prior_info.prior_centre_line= sub_centre_line;
prior_info.variance= prior_uncertainty;
%define number of lateral shifts
n_path = 5;% this has to be odd
inc = 0;
%create various roundabout configurations

orientation_exit = repmat([10:-2:-10],length([10:-2:-10]),1);
orientation_road = repmat([10:-2:-10]',1,length([10:-2:-10]));
round_configurations = cat(3,orientation_exit,orientation_road);
%circle_diam = roundabout.radius + roundabout.lane_width_round ;
%length_final = sqrt(circle_diam^2- (roundabout.lane_width_road/2)^2);
%range_height = -length_final*sind(round_configurations(:,:,1) ) - (26-length_final)*tand(round_configurations(:,:,1) +round_configurations(:,:,2));

%feasible_set = cat(3,orientation_exit(abs(range_height)<3.1),orientation_road(abs(range_height)<3.1));
%unfeasible_set = cat(3,orientation_exit(abs(range_height)>3.1),orientation_road(abs(range_height)>3.1));

%for travel = 38 : size(unfeasible_set,1)
%     orientation_exit = unfeasible_set(travel,1,1);
%     orientation_road = unfeasible_set(travel,1,2);
%     roundabout.exits = [ orientation_exit 90 180 225 270];
%     roundabout.angles = [ orientation_road 0 0 0 0];
%     roundabout.radius = 10;
%     roundabout.lane_no = 1;
%     roundabout.lane_width_round = 8;
%     roundabout.lane_width_road = 8;
%     start.exit  = 1;
%     start.distance = 50.45;
%     final.exit = 4;
%     final.distance = 15;%was 50
    
    
    %draw_roundabout(roundabout,area,55)
    
    [static_obstacle_matrix_real, line_matrix_real ,stepfine] = centre_line_environment_corr(road, start, final );
    
    
    
    plot(line_matrix_real(1,:),line_matrix_real(2,:),'-m*');
    hold on 
    plot (static_obstacle_matrix_real(1,:), static_obstacle_matrix_real(2,:),static_obstacle_matrix_real(3,:), static_obstacle_matrix_real(4,:) )
    parameters.sub_sample = round(station/stepfine);
    parameters.station = station_new;
    
    %% lattice generation
    optimal_path = motion_planner_det_lut_B(prior_info,dimension, start_pose,...
        parameters, static_obstacle_matrix_real, initial_guess_table,n_path);

end