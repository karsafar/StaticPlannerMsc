function [ ] = static_obstacles(obstacles,traj, dimension)
% static_obstalces Creates some obstacles like parked cars along the road
%   Detailed explanation goes here

%% define the pose of the obstacle
position = [ -2 30 90];

%% draw the car obstacle
draw_car(dimension, position)


%% get the coordinates of the car
rectangle = [ 0 0; dimension(2) 0; dimension(2) dimension(1); 0 dimension(1)]- [(dimension(2) - dimension(3))/2*ones(4,1) dimension(1)/2*ones(4,1) ];
front = [(dimension(2) + dimension(3))/2 0];


% rotation counter-clockwise about the origin
transformation = [cosd(position(3)) -sind(position(3)) position(1); sind(position(3)) cosd(position(3)) position(2); 0 0 1]*[rectangle' front'; ones(1,5)];

obstacle_matrix = [transformation(1,3) transformation(1,4);transformation(2,4) transformation(2,3);transformation(1,1) transformation(1,2);transformation(2,1) transformation(2,2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get the dimensions of a sparse matrix based on the station dimensions
sparseSize = 200; % there needs to ba scaling factor to adjust [-4;4] to [0:8]
%width = (round(10*(max(obstacles(4,:)) - (min(obstacles(4,:))))));

%% define the sparse matrix of the station
curbMat = zeros(sparseSize,sparseSize);

% curbMat[0;100] = [0,0]
%curbs are added as ones on the first and last columns of the matrix
scalingFactor = 2;

curbMat(scalingFactor * ((obstacles(2,:))),sparseSize / 2 + scalingFactor * (min(obstacles(1,:)))) = 1;
curbMat(scalingFactor * ((obstacles(2,:))),sparseSize / 2 + scalingFactor * (max(obstacles(3,:)))) = 1;


%% define the sparse matrix of the trajectory
%trajMat = zeros(sparseSize,sparseSize);

% scaling factor for the coordinates
dx = double(round(scalingFactor * (traj(1,:)) + sparseSize / 2));
dy = double(round(scalingFactor * ((traj(2,:)))));

% matrix with coordinates of the trajectory
trajMat = sparse(dy,dx,1,sparseSize,sparseSize);





%% create a sparse matrix of car obstacle
carMat = sparse(sparseSize,sparseSize);

% create a meshgrid of the car coordinates
[X, Y] = meshgrid(obstacle_matrix(1,1):0.5:obstacle_matrix(3,1),obstacle_matrix(2,1):0.5:obstacle_matrix(2,2));

carMat( round(scalingFactor * Y), round(sparseSize / 2 + scalingFactor * X)) = 1;
curbMat( round(scalingFactor * Y), round(sparseSize / 2 + scalingFactor * X)) = 1;
any(any(curbMat.*trajMat)) % check the collisions



end



