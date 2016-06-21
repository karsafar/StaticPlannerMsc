function [obstacle_matrix ] = static_obstacles( dimension)
% static_obstalces Creates some obstacles like parked cars along the road
%   Detailed explanation goes here

%% define the pose of the obstacle
position = [ -2 30 90];

%% draw the car obstacle 
draw_car(dimension, position)


%% get the coordinates of the car 
rectangle = [ 0 0; dimension(2) 0; dimension(2) dimension(1); 0 dimension(1)]- [(dimension(2) - dimension(3))/2*ones(4,1) dimension(1)/2*ones(4,1) ];
front = [(dimension(2) + dimension(3))/2 0];
%draw circles 
% no_of_cirles =3;
% radius = sqrt(dimension(2)^2/(4*no_of_cirles^2) + dimension(1)^2/4 );
% d = 2*sqrt(radius^2 -dimension(1)^2/4);
% centres = [d/2 0; d+d/2 0; 2*d+d/2 0]- [(dimension(2) - dimension(3))/2*ones(3,1) zeros(3,1) ];



% rotation counter-clockwise about the origin
transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front'; ones(1,5)];

obstacle_matrix = [transformation(1,3) transformation(1,4);transformation(2,4) transformation(2,3);transformation(1,1) transformation(1,2);transformation(2,1) transformation(2,2)];




end

