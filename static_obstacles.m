function [no_of_coll, positions] = static_obstacles(obstacles,traj, dimension,car_pose)
% static_obstalces Creates some obstacles like parked cars along the road
%   Detailed explanation goes here


%% Initialisation of the occupancy grid
% get the dimensions of a sparse matrix based on the station dimensions
sparseSize = 200; % there needs to ba scaling factor to adjust [-4;4] to [0:8]

% define the sparse matrix of the station
curbMat = sparse(sparseSize,sparseSize);

% curbMat[0;100] is a coordinate [0,0]
% curbs are added as ones on the first and last columns of the matrix
scalingFactor = 2;

% Initialize borders of the road (curbs)
curbMat(round(scalingFactor * ((obstacles(2,:)))),sparseSize / 2 + scalingFactor * (min(obstacles(1,:)))) = 1;
curbMat(round(scalingFactor * ((obstacles(2,:)))),sparseSize / 2 + scalingFactor * (max(obstacles(3,:)))) = 1;


%% get the coordinates of the car
rectangle = [ 0 0; dimension(2) 0; dimension(2) dimension(1); 0 dimension(1)]- [(dimension(2) - dimension(3))/2*ones(4,1) dimension(1)/2*ones(4,1) ];
front = [(dimension(2) + dimension(3))/2 0];

for i = 1:size(car_pose,1)
    [curbMat] = createCarObstacle (car_pose(i,:), curbMat,rectangle, front,scalingFactor,sparseSize);
end

%% define the sparse matrix of the trajectory
trajMat = sparse(sparseSize,sparseSize);

%% trajectories have to be widened to the size of the car
traj(3,:) = traj(3,:)*180/pi;
for i = 1:length(traj)
    % adding a width to the
    %transformation1 = [cosd(traj(3,i)) -sind(traj(3,i)) traj(1,i); sind(traj(3,i)) cosd(traj(3,i)) traj(2,i); 0 0 1]*[rectangle' front'; ones(1,5)];
    %obstacle_matrix1 = [transformation1(1,3) transformation1(1,4);transformation1(2,4) transformation1(2,3);transformation1(1,1) transformation1(1,2);transformation1(2,1) transformation1(2,2)];
    
    % call the function to create a car obstacle
    [ obstacle_matrix1 ] = createObstacle(traj(:,i),rectangle, front);
    
    [X1, Y1] = meshgrid(obstacle_matrix1(1,1):0.5:obstacle_matrix1(3,1),obstacle_matrix1(2,1):0.5:obstacle_matrix1(2,2));
    % draw_car(dimension, traj(:,i))
    trajMat( ceil(scalingFactor * Y1), round(sparseSize / 2 + scalingFactor * X1)) = 1;
end



%% check the collisions
no_of_coll = any(any(curbMat.*trajMat));

%% finding intersection points and scaling them back
[row, col] = find(curbMat.*trajMat);
positions = [row/scalingFactor, (col - sparseSize / 2)/scalingFactor] ;




% scaling should be done with higher precision



end


function [curbMat] = createCarObstacle (position, curbMat,rectangle, front,scalingFactor,sparseSize)
% %% draw the car obstacle if it is within the station
% if min(traj(2,:)) <= position(2) && max(traj(2,:) >= position(2))
%     draw_car_obstacle(dimension, position)
% end

% call the function to create a car obstacle
[ obstacle_matrix ] = createObstacle(position,rectangle, front);

% create a meshgrid of the car coordinates
[X, Y] = meshgrid(obstacle_matrix(1,1):0.5:obstacle_matrix(3,1),obstacle_matrix(2,1):0.5:obstacle_matrix(2,2));

% add the car obstacle to the obstacle matrix
curbMat( round(scalingFactor * Y), round(sparseSize / 2 + scalingFactor * X)) = 1;
end


function [ obstacle_matrix ] = createObstacle(position,rectangle, front)

% rotation counter-clockwise about the origin
transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front'; ones(1,5)];

obstacle_matrix = [transformation(1,3) transformation(1,4);transformation(2,4) transformation(2,3);transformation(1,1) transformation(1,2);transformation(2,1) transformation(2,2)];
end



